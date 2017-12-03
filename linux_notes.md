## scp 使用方式

scp [帳號@來源主機]:來源檔案 [帳號@目的主機]:目的檔案

```
# 從本地端複製到遠端
scp /path/file1 myuser@192.168.0.1:/path/file2
```

```
# 從遠端複製到本地端
scp myuser@192.168.0.1:/path/file2 /path/file1
```
```
# 複製目錄
scp -r /path/folder1 myuser@192.168.0.1:/path/folder2
```

```
# 保留檔案時間與權限
scp -p /path/file1 myuser@192.168.0.1:/path/file2
```

```
# 資料壓縮
scp -C /path/file1 myuser@192.168.0.1:/path/file2
```


## virtualevn 使用方式
安裝
```
sudo pip install virtualenv
```
創建虛擬環境
```
virtualenv vspace
```
或是指定python版本
```
virtualenv -p /usr/bin/python3 vspace
```

進入虛擬環境
```
source vspace/bin/activate
```

離開虛擬環境
```
deactivate
```

