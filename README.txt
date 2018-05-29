** Start with '$ source setup_paths' 

** For MpiSintel, images and the flow outputs have been resized to 448x1024 (from 436x1024) to avoid concatenation error! Changes done in frame_utils.py

** Changes carried out in flow_utils.py for saving color-coded flow results. 

** To run on entire MpiSintel Clean dataset
python main.py --inference --model FlowNet2 --save_flow --inference_dataset MpiSintelClean --inference_dataset_root ./datasets/MPI-Sintel-complete/training --resume ./checkpoints/FlowNet2_checkpoint.pth.tar --skip_training --skip_validation
(Add CUDA_VISIBLE_DEVICES=2 before the run command if "run out of memory")

** To run on a particular directory, and save results in a directory say './results'
python main.py --inference --model FlowNet2 --save_flow --inference_dataset ImagesFromFolder --inference_dataset_root ./datasets/MPI-Sintel-complete/test/clean/ambush_1 --save ./results --resume ./checkpoints/FlowNet2_checkpoint.pth.tar --skip_training --skip_validation
