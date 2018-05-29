This is a slightly modified version of the original flownet2-pytorch code available from here https://github.com/NVIDIA/flownet2-pytorch
The modifications are only carried out to make it compatible to python 3.6.3. Plus a few modifications to save color-coded flow maps (.png images) directly (the code would only generate .flo files initially)

To run a sample test, download the checkpoints (pre-trained models from the URL mentioned above) and place them in the ./checkpoints/ directory; similarly add the datasets into the ./datasets/directory

Please go through the README.txt file to look for a few sample run commands. 
