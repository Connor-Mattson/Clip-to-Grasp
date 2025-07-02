# RoboCLIP
Simulated Grasping from Multi-Modal Grounding (CLIP)


### TLDR
Just run this
```
pip install -r requirements.txt
TODO: Final script
```


## Setup

Install all requirements
```
pip install -r requirements.txt
```
I ran all experiments are run on a Macbook Pro M3 Max. Since we only require model inference (not training), the experiments are all run on the CPU. 

## Experiments

### Testing CLIP
To test that the clip model is working you can run
```
python examples/clip_example.py
```

This will embed an image of (my dog)[media/dog.jpeg] to the model and determine the similarity to "A Diagram", "A Dog", and "A Cat", you should see output similar to:
```
Label probs: [('a diagram', 0.012420654296875), ('a dog', 0.986328125), ('a cat', 0.0014600753784179688)]
```

### Loading a PyBullet Scene


