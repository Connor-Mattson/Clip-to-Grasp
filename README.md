# RoboCLIP
_Authors: Connor Mattson (@Connor-Mattson)_

Simulated Grasping from Multi-Modal Grounding (CLIP)

### TL;DR
Want quick results? Just run this
```
pip install -r requirements.txt
TODO: Final script
```

Otherwise, feel free to keep reading.

## Setup
Install all requirements
```
pip install -r requirements.txt
```
I ran all experiments on a Macbook Pro M3 Max. Since we only require model inference (not training), the experiments are all run on the CPU. 

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

## Acknowledgements
- Thanks to @kwonathan for the [great repo with URDFs for the YCB dataset](https://github.com/kwonathan/ycb_urdfs/tree/main).
- Thanks to OpenAI for publically releasing the weights for their CLIP model, which empowered this project with vision + language capabilities.