import torch
import clip
from PIL import Image

# Load the CLIP model
device = "mps" if torch.backends.mps.is_available() else "cpu"
model, preprocess = clip.load("ViT-B/32", device=device)

# Pull a picture of my dog for classification
image = preprocess(
    Image.open(
        "media/dog.jpg"
    )
).unsqueeze(0).to(device)

# Tokenize the possible labels
labels = ["a diagram", "a dog", "a cat"]
text = clip.tokenize(labels).to(device)

# Encode the image and text
with torch.no_grad():
    image_features = model.encode_image(image)
    text_features = model.encode_text(text)

    logits_per_image, logits_per_text = model(image, text)
    probs = logits_per_image.softmax(dim=-1).cpu().numpy()

# Output the probabilities for each label
print("Label probs:", list(zip(labels, probs[0].tolist())))
