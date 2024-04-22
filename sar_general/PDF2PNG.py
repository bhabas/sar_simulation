import os
from pdf2image import convert_from_path

def convert_pdf_to_png(directory, dpi=300):
    for root, dirs, files in os.walk(directory):
        for file in files:
            if file.lower().endswith(".pdf") and 'notext' in file.lower():
                pdf_path = os.path.join(root, file)
                images = convert_from_path(pdf_path, dpi=dpi)
                for i, image in enumerate(images):
                    image_path = f"{pdf_path[:-4]}.png"
                    image.save(image_path, 'PNG')
                    print(f"Saved: {image_path}")

# Replace 'your/directory/path' with the path to the directory containing your PDF files
# You can also specify the desired DPI (dots per inch), e.g., dpi=150
# convert_pdf_to_png('your/directory/path', dpi=150)
convert_pdf_to_png('/home/bhabas/catkin_ws/src/sar_simulation/sar_projects/DeepRL/TB_Logs/', dpi=150)
