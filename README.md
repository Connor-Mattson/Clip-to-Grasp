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

### Testing CLIP
To test that the clip model is working you can run
```bash
python -m examples.clip_example
```

This will embed an image of [my dog](media/dog.jpg) to the model and determine the similarity to "A Diagram", "A Dog", and "A Cat", you should see output similar to:
```
Label probs: [('a diagram', 0.012420654296875), ('a dog', 0.986328125), ('a cat', 0.0014600753784179688)]
```

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


## Acknowledgements
- Thanks to @kwonathan for the [great repo with URDFs for the YCB dataset](https://github.com/kwonathan/ycb_urdfs/tree/main).
- Thanks to OpenAI for publically releasing the weights for their CLIP model, which empowered this project with vision + language capabilities.