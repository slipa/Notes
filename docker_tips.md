Docker Tips
==========
### How to mount a folder of host system to an running container
運行中的container無法掛載host中的資料夾。須重新啟用該image時一併掛載，如果需要保存目前container狀態，將其另存成一新的image後重新啟用。

for example:
```
$docker commit <container ID> <new image name>
$docker run -ti -v "$PWD/dir1":/dir1 -v "$PWD/dir2":/dir2 <new image name> /bin/bash
```

### Save Container as an image tar file, and reload it later
1. 使用以下的指令將 Docker Image 存檔出一個檔案如下

```
$ docker save -o mytomcat.tar mytomcat
```

參數說明如下：  
-o: 輸出檔案  
mytomcat 是 Docker Image 的名稱

2. 把檔案 Load 到 Docker 的指令如下

```
$ docker load -i mytomcat.tar
```
參數說明如下：  
-i: 放要 import 的檔案名稱

3. 確認image有被加入Docker中
```
docker image ls
```