import os
import os.path as osp

import torch
import clip
import numpy as np
from PIL import Image


class CompleteTopologyGraph():
    """Implementation of complete topology graph described in LM-Nav (Shah et al., 2023)"""
    def __init__(self, data_dir):
        self._img_dir = osp.join(data_dir, 'rgb')
        self._pose_dir = osp.join(data_dir, 'world_pose_w_timestamp')
        self._img_feat_dir = osp.join(data_dir, 'rgb_feat')

        self._imgs = [] # list of PIL.Image
        self._poses = [] # list of (x, y, z, q)
        self._img_features = None # np array with shape (# of images, img feature dim)
        
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        print(f"device:{self.device}")
        self.clip_version = "ViT-B/32"
        self.clip_model = None
        self.preprocess = None
        
        self._load_clip_model()
        self._cache_pose()
        self._cache_image_feature()
        self._cache_image()
    
    def get_text_feats(self, in_text: list, clip_model, clip_feat_dim, batch_size=64):
        if torch.cuda.is_available() :
            text_tokens = clip.tokenize(in_text).cuda()
        else :
            text_tokens = clip.tokenize(in_text)
        text_id = 0
        text_feats = np.zeros((len(in_text), clip_feat_dim), dtype=np.float32)
        while text_id < len(text_tokens):  # Batched inference.
            batch_size = min(len(in_text) - text_id, batch_size)
            text_batch = text_tokens[text_id : text_id + batch_size]
            with torch.no_grad():
                batch_feats = clip_model.encode_text(text_batch).float()
            batch_feats /= batch_feats.norm(dim=-1, keepdim=True)
            batch_feats = np.float32(batch_feats.cpu())
            text_feats[text_id : text_id + batch_size, :] = batch_feats
            text_id += batch_size
        return text_feats
        
    def _cache_image(self):
        for img in sorted(os.listdir(self._img_dir), key=lambda x: int(x.split('.')[0])):
            img_path = osp.join(self._img_dir, img)
            img = Image.open(img_path)
            self._imgs.append(img)
        
    def _cache_pose(self):
        for pos in sorted(os.listdir(self._pose_dir), key=lambda x: int(x.split('.')[0])):
            pos_path = osp.join(self._pose_dir, pos)
            with open(pos_path, 'r') as f:
                text = f.read()
                row = [float(x) for x in text.split()[1:]] # ignore time stamp
                self._poses.append(row)
    
    def _cache_image_feature(self):
        feats = []
        for img_feat in sorted(os.listdir(self._img_feat_dir), key=lambda x: int(x.split('.')[0])):
            img_feat_path = osp.join(self._img_feat_dir, img_feat)
            img_feat = np.load(img_feat_path)
            feats.append(img_feat[0])
        self._img_features = np.stack(feats, axis=0) # (N, feat_dim)
        
    def _load_clip_model(self):
        """Load CLIP related variables."""
        self.clip_feat_dim = {'RN50': 1024, 'RN101': 512, 'RN50x4': 640, 'RN50x16': 768,
                            'RN50x64': 1024, 'ViT-B/32': 512, 'ViT-B/16': 512, 'ViT-L/14': 768}[self.clip_version]
            
        clip_model, preprocess = clip.load(self.clip_version) 
        self.clip_model = clip_model
        self.clip_model.to(self.device).eval()
        self.preprocess = preprocess
     
    def __call__(self, landmarks, vis=False):
        """Return the most likely images for given landmark list and images' poses."""
        if isinstance(landmarks, str):
            landmarks = [landmarks]
        
        text_feat = self.get_text_feats(landmarks, self.clip_model, self.clip_feat_dim)

        scores = self._img_features @ text_feat.T # (n_imgs, n_texts)
        scores = scores.squeeze()
        max_indices = np.argmax(scores, axis=0)
        if isinstance(max_indices, np.int64):
            max_indices = [max_indices]
        
        poses = [self._poses[idx] for idx in max_indices]
        imgs = [self._imgs[idx] for idx in max_indices]
        
        if vis:
            return poses, imgs
        else:
            return poses