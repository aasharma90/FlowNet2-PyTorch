#!/bin/bash

# Inference on Our_Oxford_RobotCar dataset with pre-trained FlowNet2-C
for i in {1..10}
do
	python main.py --inference --model FlowNet2C --save_flow --inference_dataset ImagesFromFolder --inference_dataset_root './datasets/Our_Oxford_RobotCar/i'$i'/' --save './results-C/i'$i'/' --resume ./checkpoints/FlowNet2-C_checkpoint.pth.tar --skip_training --skip_validation
	python main.py --inference --model FlowNet2C --save_flow --inference_dataset ImagesFromFolder --inference_dataset_root './datasets/Our_Oxford_RobotCar/i'$i'_LIME/' --save './results-C/i'$i'_LIME/' --resume ./checkpoints/FlowNet2-C_checkpoint.pth.tar --skip_training --skip_validation
done

# Inference on Our_Oxford_RobotCar dataset with pre-trained FlowNet2
for i in {1..10}
do
	python main.py --inference --model FlowNet2 --save_flow --inference_dataset ImagesFromFolder --inference_dataset_root './datasets/Our_Oxford_RobotCar/i'$i'/' --save './results/i'$i'/' --resume ./checkpoints/FlowNet2_checkpoint.pth.tar --skip_training --skip_validation
	python main.py --inference --model FlowNet2 --save_flow --inference_dataset ImagesFromFolder --inference_dataset_root './datasets/Our_Oxford_RobotCar/i'$i'_LIME/' --save './results/i'$i'_LIME/' --resume ./checkpoints/FlowNet2_checkpoint.pth.tar --skip_training --skip_validation
done