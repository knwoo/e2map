import os
import os.path as osp
import argparse

import torch
import clip
import numpy as np
from PIL import Image
from tqdm import tqdm


def get_img_feats(img, preprocess, clip_model):
    img_pil = Image.fromarray(np.uint8(img))
    img_in = preprocess(img_pil)[None, ...]
    with torch.no_grad():
        if torch.cuda.is_available() :
            img_feats = clip_model.encode_image(img_in.cuda()).float()
        else :
            img_feats = clip_model.encode_image(img_in).float()
    img_feats /= img_feats.norm(dim=-1, keepdim=True)
    img_feats = np.float32(img_feats.cpu())
    return img_feats


def encode_image_features_with_clip(data_dir, preprocess, clip_model):
    img_dir = osp.join(data_dir, 'rgb')
    img_feat_save_dir = osp.join(data_dir, 'rgb_feat')
    
    if not osp.exists(img_feat_save_dir):
        os.makedirs(img_feat_save_dir)
    
    for i, img in enumerate(tqdm(sorted(os.listdir(img_dir), key=lambda x: int(x.split('.')[0])), desc='save image features..')):
        img_path = osp.join(img_dir, img)
        img = Image.open(img_path)
        img_feat = get_img_feats(img, preprocess, clip_model) #(1, feat_dim)
        save_path = osp.join(img_feat_save_dir, f'{i+1}.npy')
        np.save(save_path, img_feat)
    
    assert len(os.listdir(img_dir)) == len(os.listdir(img_feat_save_dir)), "the number of items in both directory should be same."
    
    
if __name__ == '__main__':
    src_dir = osp.dirname(osp.dirname(osp.abspath(__file__)))
    workspace_dir = osp.dirname(src_dir)
    data_dir = osp.join(workspace_dir, 'data_209')
    
    parser = argparse.ArgumentParser(description="Argument parser for topology graph")
    parser.add_argument("--data_dir", default=f'{data_dir}', help="directory that includes rgb images and pose.")
    args = parser.parse_args()

    device = "cuda" if torch.cuda.is_available() else "cpu"
    print("Device: ", device)
    clip_version = "ViT-B/32"
    clip_feat_dim = {'RN50': 1024, 'RN101': 512, 'RN50x4': 640, 'RN50x16': 768,
                    'RN50x64': 1024, 'ViT-B/32': 512, 'ViT-B/16': 512, 'ViT-L/14': 768}[clip_version]
    print("Loading CLIP model...")
    clip_model, preprocess = clip.load(clip_version)
    
    encode_image_features_with_clip(args.data_dir, preprocess, clip_model)
    