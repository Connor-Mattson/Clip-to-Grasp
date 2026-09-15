import torch
import clip
from PIL import Image
import numpy as np

# Load the CLIP model
device = "mps" if torch.backends.mps.is_available() else "cpu"
model, preprocess = clip.load("ViT-B/32", device=device)

image_paths = [
    "media/cropped_ycb/0.png",
    "media/cropped_ycb/1.png",
    "media/cropped_ycb/2.png",
    "media/cropped_ycb/3.png",
]

images = []
image_feature_list = []

# Load in all images
for img_path in image_paths:
    img = Image.open(
        img_path
    )
    images.append(img)

# Encode all images
for img in images:
    image = preprocess(
        img
    ).unsqueeze(0).to(device)
    with torch.no_grad():
        image_features = model.encode_image(image)
        image_features = image_features.cpu()
        image_feature_list.append(image_features)

# Tokenize user input
user_input = input("Enter a query: ")
# user_input = "a banana"
labels = [user_input]
text = clip.tokenize(labels).to(device)

# Encode the image and text
with torch.no_grad():
    text_features = model.encode_text(text).cpu()

# Compute the cosine similarity between the image and text features
similarities = []
for i, image_feature in enumerate(image_feature_list):
    similarity = torch.nn.functional.cosine_similarity(image_feature, text_features, dim=1).item()
    print(f"Image {i} similarity: {similarity}")
    similarities.append(similarity)

# Display the best match
best_match = np.argmax(similarities)
print(f"Best match: {best_match}")
print(f"Showing image {best_match}")
images[best_match].show()
