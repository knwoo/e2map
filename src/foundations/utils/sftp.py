import pysftp
import os


class SFTP:
    def __init__(self, hostname, username, password, port=22):
        self.connection = None
        self.hostname = hostname
        self.username = username
        self.password = password
        self.port = port
        self.listdir = []
        self.listdir_attr = []
        
    def connect(self):
        try:
            self.connection = pysftp.Connection(
                host=self.hostname,
                username=self.username,
                password=self.password,
                port=self.port,
            )
        except Exception as err:
            raise Exception(err)
        finally:
            print(f"Connected to {self.hostname} as {self.username}.")
            
    def disconnect(self):
        self.connection.close()
        print(f"Disconnected from host {self.hostname}.")
        
    def get_listdir(self, remote_path):
        for obj in self.connection.listdir(remote_path):
            self.listdir.append(obj)

    def get_listdir_attr(self, remote_path):
        for attr in self.connection.listdir_attr(remote_path):
            self.listdir_attr.append(attr)
            
    def upload(self, source_local_path, remote_path):
        try:
            print(
                f"Uploading to {self.hostname} as {self.username} (source_local_path: {source_local_path}) -> (remote_path: {remote_path}))"
            )
            
            self.connection.put(source_local_path, remote_path)
            print("Upload complteted!")
        
        except Exception as err:
            raise Exception(err)
        
    def download(self, remote_path, target_local_path):
        try:
            print(
                f"Downloading from {self.hostname} as {self.username} (remote_path: {remote_path}) -> (target_local_path: {target_local_path}))"
            )
            
            path, _ = os.path.split(target_local_path)
            if not os.path.isdir(path):
                try:
                    os.makedirs(path)
                except Exception as err:
                    raise Exception(err)
            
            self.connection.get(remote_path, target_local_path)
            print("Download completed!")
        
        except Exception as err:
            raise Exception(err)