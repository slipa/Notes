

## 安裝Jupyter Notebooks
```
$sudo apt install nodejs npm
$sudo apt install python3-pip
$pip3 install jupyter jupyterlab
```

## Make Jupyter as a service

### Find path of jupyter
```
$ which jupyter
/home/robot/.local/bin/jupyter
```

### Create /etc/systemd/system/jupyter.service

```
$sudo nano /et/csystemd/system/jupyter.service
```
paste following content in jupyter.service
```
[Unit]
Description=Jupyter Lab

[Service]
Type=simple
PIDFile=/run/jupyter.pid
ExecStart=/home/robot/.local/bin/jupyter lab --config=/home/robot/.jupyter/jupyter_notebook_config.py
User=robot
Group=robot
WorkingDirectory=/home/robot/Notebooks
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

```
$sudo systemctl enable jupyter.service
$sudo systemctl daemon-reload  ## jupyter.service內容有更動時須執行此指令
$sudo systemctl start jupyter.service
```