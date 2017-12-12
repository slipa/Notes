
Date: 2017/12/05

#### 環境建置
啟動Keras Container

```
$ docker run --name keras -it -p=8888:8888 -v=/home/samuel/playground/dl/keras/notebooks:/srv gw000/keras-full bash
```
進入Container之後，啟動Jupyter Notebook:
```
# jupyter notebook --ip=0.0.0.0 --port=8888 --allow-root
```


