import os
import shutil
import sys

# Configuration variable

# This variable is the path to the solidworks output folder of urdf files
source_dir = '/home/sxm/Project/ros/test_urdf/'

# This variable is the package path of ros2
target_dir = '/home/sxm/Project/ros/test_urdf_tool_ws/src/test_urdf_tool/'

# This variable is the package-name of ros2
package_name = "test_urdf_tool"

# This variable is the solidworks output folder name
output_folder_name = "test_urdf"


def run_command_dir(command_dir, command):
    os.system("cd " + command_dir + " && " + command)


def replace_str(file, old_str, new_str):
    file_data = ""
    with open(file, "r", encoding="utf-8") as f:
        for line in f:
            if old_str in line:
                line = line.replace(old_str, new_str)
            file_data += line
    with open(file, "w", encoding="utf-8") as f:
        f.write(file_data)


if __name__ == '__main__':
    # Create folders
    run_command_dir(target_dir, "mkdir launch meshes meshes/collision meshes/visual urdf")

    # Copy files
    # Copy stl files
    run_command_dir(source_dir, "cp -r ./meshes/* " + target_dir + "meshes/visual")
    run_command_dir(source_dir, "cp -r ./meshes/* " + target_dir + "meshes/collision")
    # Copy urdf files
    run_command_dir(source_dir, "cp  ./urdf/" + output_folder_name + ".urdf " + target_dir + "urdf/")

    # replace files
    os.system("cp -f ./replace_files/setup.py " + target_dir)
    os.system("cp -f ./replace_files/package.xml " + target_dir)
    os.system("cp -f ./replace_files/launch.py " + target_dir + "launch")

    # Change file content
    # launch.py
    replace_str(target_dir + "launch/launch.py", "lesson_urdf", package_name)
    replace_str(target_dir + "launch/launch.py", "planar_3dof.urdf", output_folder_name + ".urdf")
    # setup.py
    replace_str(target_dir + "setup.py", "lesson_urdf", package_name)
    # package.xml
    replace_str(target_dir + "package.xml", "lesson_urdf", package_name)
    # urdf files
    replace_str(target_dir + "urdf/" + output_folder_name + ".urdf", output_folder_name + "/meshes",
                package_name + "/meshes/visual")

    # Insert base_footprint
    keyword = "name=\"" + output_folder_name + "\">"
    str = ""
    with open("./replace_files/insert_content.txt", "r", encoding="utf-8") as f:
        str = f.read()

    file = open(target_dir + "/urdf/" + output_folder_name + ".urdf", 'r')
    content = file.read()
    post = content.find(keyword)
    if post != -1:
        content = content[:post + len(keyword)] + "\n" + str + content[post + len(keyword):]
        file = open(target_dir + "/urdf/" + output_folder_name + ".urdf", "w")
        file.write(content)
    file.close()

    print("conversion success!")
