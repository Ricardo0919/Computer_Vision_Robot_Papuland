import os
import shutil

def merge_split(split, dataset1, dataset2, merged):
    img_dst = os.path.join(merged, split, "images")
    lbl_dst = os.path.join(merged, split, "labels")
    os.makedirs(img_dst, exist_ok=True)
    os.makedirs(lbl_dst, exist_ok=True)

    for dataset in [dataset1, dataset2]:
        img_src = os.path.join(dataset, split, "images")
        lbl_src = os.path.join(dataset, split, "labels")

        for filename in os.listdir(img_src):
            base, ext = os.path.splitext(filename)
            new_name = base
            counter = 1

            # Asegura que no se repita el nombre
            while os.path.exists(os.path.join(img_dst, new_name + ext)):
                new_name = f"{base}_{counter}"
                counter += 1

            shutil.copyfile(os.path.join(img_src, filename), os.path.join(img_dst, new_name + ext))
            shutil.copyfile(os.path.join(lbl_src, base + ".txt"), os.path.join(lbl_dst, new_name + ".txt"))

# Rutas absolutas o relativas desde donde estás
dataset1 = "datasetOriginal"
dataset2 = "datasetTrack"
merged = "datasetMerged"

for split in ["train", "valid", "test"]:
    merge_split(split, dataset1, dataset2, merged)

print("✅ Merge completado. Revisa datasetMerged/")
