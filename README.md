# RoboCLIP
_Authors: Connor Mattson (@Connor-Mattson)_

Simulated Grasping from Multi-Modal Grounding (CLIP)

### TL;DR
Want quick results? Just run this
```
conda env create -f environment.yml
conda activate roboCLIP
pip install -r requirements.txt
python -m main
```

Otherwise, feel free to keep reading.

## Setup
Create the conda environment (provides PyBullet), then install the Python requirements into it
```
conda env create -f environment.yml
conda activate roboCLIP
pip install -r requirements.txt
```
The environment file was exported on macOS. I ran all experiments on a Macbook Pro M3 Max. Since we only require model inference (not training), the experiments are all run on the CPU. 

## Experiments

### Loading a PyBullet Scene with Franka Panda + IK
Run the following script to test the control and inverse kinematics solver of the Franka Panda robot.

```bash
python -m examples.panda_ik
```

If your pybullet is setup correctly, you should see the OpenGL gui open and the robot pick up a rigid block in the environment.

To see an example of hard-coded pick-n-place with several scene objects, you can run 

```bash
python -m examples.ycb_scene
```
After initialization, you'll see the following prompt in the console: `Enter the object to grasp:`. Select one of the objects in the scene by responsing with the name in lowercase letters, e.g. "apple", "banana", "soup", "mug".

### Extract Object Images from end-effector camera
You can test if the virtual camera works in the scene using the following commmand

```bash
python -m examples.ee_camera_example
```

The robot will not move in this demo, you'll just see the objects spawn in and the matplotlib GUI display.

### Draw BB on the EE image
```bash
python -m examples.object_extraction_example
```

After the renderer loads everything in, you'll see the BB cropped objects from the EE perspective appear in a pop-up window. After closing, you'll see each of the individual crops of the photo for each object, which will be injested by CLIP.

---

### Testing CLIP
To test that the clip model is working you can run
```bash
python -m examples.clip_example
```

This will embed an image of [my dog](media/dog.jpg) to the model and determine the similarity to "A Diagram", "A Dog", and "A Cat", you should see output similar to:
```
Label probs: [('a diagram', 0.012420654296875), ('a dog', 0.986328125), ('a cat', 0.0014600753784179688)]
```

Then, you can test CLIP alignment with an open-vocabluary query on the 4 extracted images using
```bash 
python -m examples.clip_repr_example
```

---

### End-to-End: Language Query to Pick-and-Place
Put everything together with
```bash
python -m main
```

After the scene settles, the robot captures an image from its end-effector camera and crops out each object. At `Enter a query:`, describe an object in plain language, e.g. "a banana", "something yellow", "a mug". CLIP picks the crop that best matches the query, and the Panda grasps that object from its current pose and places it on the plate.

If no crop is similar enough to the query (e.g. "a giraffe"), the script prints `No object matches ...` and exits without moving. The cutoff is `MIN_SIMILARITY` in `main.py`, measured on this scene: it rejects clearly unrelated queries, but a query for a similar-looking object that isn't there (e.g. "a hammer") can still pick the closest match. Re-check it if you change the objects or camera.

## Acknowledgements
- Thanks to @kwonathan for the [great repo with URDFs for the YCB dataset](https://github.com/kwonathan/ycb_urdfs/tree/main).
- Thanks to OpenAI for publically releasing the weights for their CLIP model, which empowered this project with vision + language capabilities.