import os
import shutil
import sys
import tkinter as tk
from tkinter import filedialog
from tkinter import messagebox
import xml.etree.ElementTree as ET

# Configuration variable
def get_directory(title):
    path = filedialog.askdirectory(title=title)
    # Ensure the path ends with a forward slash
    if not path.endswith("/"):
        path += "/"
    return path

def run_command_dir(command_dir, command):
    os.system(f"cd {command_dir} && {command}")

def replace_str(file, old_str, new_str):
    file_data = ""
    with open(file, "r", encoding="utf-8") as f:
        for line in f:
            if old_str in line:
                line = line.replace(old_str, new_str)
            file_data += line
    with open(file, "w", encoding="utf-8") as f:
        f.write(file_data)

# Function to modify URDF
def modify_urdf(urdf_path, package_name):
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    
    # Find all fixed joints and change them to continuous
    for joint in root.findall(".//joint"):
        if joint.get("type") == "fixed":
            joint.set("type", "continuous")
            # Change z-axis in origin from 0 to 1
            origin = joint.find("axis")
            if origin is not None:
                z = origin.get("xyz").split()[2]
                if z == '0':
                    origin.set("xyz", origin.get("xyz")[:origin.get("xyz").rfind('0')] + '1')
    tree.write(urdf_path)

def replace_model_with_package(sdf_path):
    try:
        with open(sdf_path, 'r', encoding='utf-8') as file:
            content = file.read()
        content = content.replace('model://', 'package://')
        with open(sdf_path, 'w', encoding='utf-8') as file:
            file.write(content)
        print(f"Successfully replaced 'model://' with 'package://' in {sdf_path}")
    except Exception as e:
        print(f"Error replacing 'model://' with 'package://' in {sdf_path}: {e}")

# Function to modify SDF based on original URDF
def modify_sdf(sdf_path, original_urdf_path):
    tree_urdf = ET.parse(original_urdf_path)
    root_urdf = tree_urdf.getroot()
    fixed_joints = [joint.get("name") for joint in root_urdf.findall(".//joint") if joint.get("type") == "fixed"]
    
    tree_sdf = ET.parse(sdf_path)
    root_sdf = tree_sdf.getroot()
    
    # Change revolute joints to fixed if they were fixed in the original URDF
    for joint in root_sdf.findall(".//joint"):
        if joint.get("name") in fixed_joints and joint.get("type") == "revolute":
            joint.set("type", "fixed")
    
    tree_sdf.write(sdf_path)
    # Replace "model://" with "package://" in the SDF file
    replace_model_with_package(sdf_path)

# GUI application
class ConversionApp:
    def __init__(self, root):
        self.root = root
        self.root.title("SolidWorks to ROS2 Conversion")

        self.source_dir = ""
        self.target_dir = ""

        # Create and place labels and buttons
        self.source_label = tk.Label(root, text="Source Directory (SolidWorks URDF Output):")
        self.source_label.pack(pady=5)

        self.source_button = tk.Button(root, text="Select Source Folder", command=self.select_source)
        self.source_button.pack(pady=5)

        self.target_label = tk.Label(root, text="Target Directory (ROS2 Package Path):")
        self.target_label.pack(pady=5)

        self.target_button = tk.Button(root, text="Select Target Folder", command=self.select_target)
        self.target_button.pack(pady=5)

        self.convert_button = tk.Button(root, text="Start Conversion", command=self.start_conversion)
        self.convert_button.pack(pady=20)

    def select_source(self):
        self.source_dir = get_directory("Select the folder generated from SolidWorks (URDF Output)")
        self.source_label.config(text=f"Source Directory: {self.source_dir}")

    def select_target(self):
        self.target_dir = get_directory("Select the ROS2 package folder")
        self.target_label.config(text=f"Target Directory: {self.target_dir}")

    def start_conversion(self):
        if not self.source_dir or not self.target_dir:
            messagebox.showerror("Error", "Please select both source and target directories.")
            return

        package_name = self.target_dir.split("/")[-2]
        output_folder_name = self.source_dir.split("/")[-2]

        print("Source Directory: " + self.source_dir)
        print("Target Directory: " + self.target_dir)
        print("Package Name: " + package_name)
        print("Output Folder Name: " + output_folder_name)

        # Create folders
        run_command_dir(self.target_dir, "mkdir launch meshes meshes/collision meshes/visual urdf")

        # Copy files
        # Copy stl files
        run_command_dir(self.source_dir, f"cp -r ./meshes/* {self.target_dir}meshes/visual")
        run_command_dir(self.source_dir, f"cp -r ./meshes/* {self.target_dir}meshes/collision")
        # Copy urdf files
        run_command_dir(self.source_dir, f"cp  ./urdf/{output_folder_name}.urdf {self.target_dir}urdf/")

        # replace files
        os.system(f"cp -r ./replace_files/world {self.target_dir}")
        os.system(f"cp -r ./replace_files/config {self.target_dir}")
        os.system(f"cp -f ./replace_files/setup.py {self.target_dir}")
        os.system(f"cp -f ./replace_files/package.xml {self.target_dir}")
        os.system(f"cp -f ./replace_files/launch.py {self.target_dir}launch")
        os.system(f"cp -f ./replace_files/gz_simulator_launch.py {self.target_dir}launch")

        # Change file content
        # launch.py
        replace_str(f"{self.target_dir}launch/launch.py", "lesson_urdf", package_name)
        replace_str(f"{self.target_dir}launch/gz_simulator_launch.py", "lesson_urdf", package_name)
        replace_str(f"{self.target_dir}launch/launch.py", "planar_3dof.urdf", f"{output_folder_name}.urdf")
        replace_str(f"{self.target_dir}launch/gz_simulator_launch.py", "planar_3dof.urdf", f"{output_folder_name}.urdf")
        # setup.py
        replace_str(f"{self.target_dir}setup.py", "lesson_urdf", package_name)
        # package.xml
        replace_str(f"{self.target_dir}package.xml", "lesson_urdf", package_name)
        # urdf files
        replace_str(f"{self.target_dir}urdf/{output_folder_name}.urdf", f"{output_folder_name}/meshes", f"{package_name}/meshes/visual")

        # Copy the generated URDF
        copied_urdf_path = f"{self.target_dir}urdf/{output_folder_name}_modified.urdf"
        shutil.copy(f"{self.target_dir}urdf/{output_folder_name}.urdf", copied_urdf_path)

        # Modify the copied URDF
        modify_urdf(copied_urdf_path, package_name)

        # Generate SDF from modified URDF
        os.system(f"cd {self.target_dir}urdf/ && gz sdf -p {output_folder_name}_modified.urdf > robot.sdf")

        # Modify the generated SDF
        modify_sdf(f"{self.target_dir}urdf/robot.sdf", f"{self.target_dir}urdf/{output_folder_name}.urdf")

        # Delete the copied URDF
        os.remove(copied_urdf_path)

        messagebox.showinfo("Success", "Conversion completed successfully!")

# Run the GUI
if __name__ == '__main__':
    root = tk.Tk()
    app = ConversionApp(root)
    root.mainloop()
