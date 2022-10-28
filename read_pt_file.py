import torch

# Load category and color encodings
cat_dict = torch.load('/home/thinking/detection_ws/3D-Fusion/data/output_data/000635.pt')
# cat_dict = torch.load('/home/thinking/detection_ws/hhh/1.pt')
print(len(cat_dict))
for k, v in cat_dict.items():  # k 参数名 v 对应参数值
    print(k, v)


# print(len(cat_dict[0]['occluded']))