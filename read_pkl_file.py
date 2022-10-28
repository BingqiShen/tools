import pickle

fr = open("/home/thinking/detection_ws/OpenPCDet/output/kitti_models/second/default/eval/eval_with_train/epoch_15/val/result.pkl",'rb')# open的参数是pkl文件的路径
inf = pickle.load(fr)  # 读取pkl文件的内容
print(inf)
fr.close()
