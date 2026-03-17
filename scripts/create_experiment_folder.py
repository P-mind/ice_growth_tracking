from datetime import datetime
import os

root = "./data"

name = datetime.now().strftime("%Y_%m_%d_%H_%M")

path = os.path.join(root,name)

os.makedirs(path,exist_ok=True)

print("Created experiment folder:",path)