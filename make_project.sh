# This is for making clangd finding all the includes, so that LSP find no errors for missing ros libraries
catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=1
ln -s build/compile_commands.json compile_commands.json
