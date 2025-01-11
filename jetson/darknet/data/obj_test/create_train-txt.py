import os

if os.path.exists("valid.txt"):
    os.remove("valid.txt")

f = open("valid.txt", "a")

for filename in os.listdir("."):
    if filename.endswith('.jpg'):
        f.write(f"data/obj_test/{filename}\n")
