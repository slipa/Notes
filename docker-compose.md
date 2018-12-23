# docker-compose

## Install
```
$ sudo apt install docker-compose
```

## Check docker-compose version 
```
$ docker-compose version
```

## 啟動/停止 docker-compose

**啟動**: 進入docker-compose.yml所在目錄，執行docker-compose up
```
$ cd <path/to/docker-compose.yml>
$ docker-compose up
```
**停止**: 進入docker-compose.yml所在目錄，執行docker-compose down
```
$ cd <path/to/docker-compose.yml>
$ docker-compose down
```

## Compose file version 3 
docker-compose.yml 使用yaml格式，yaml介紹可參考[Day 9 - 攥寫設定能不學嗎：yaml](https://ithelp.ithome.com.tw/articles/10193509)  

目前最新的 Compose file 為版本 3，此文件會分成三大組態設定，分別為 services、networks 以及 volumes：

**services** (top-level)

services 主要讓你定義你應用服務啟動時，用來執行的 container 相關資訊，比如說 image 是用哪個、是不是要透過 Dockerfile 先進行編譯、要不要覆寫預設 command 或是 entrypoint、環境變數為何、port 導出與對應等等的，這些參數多半在 docker run 指令有相對應的參數。可以參照著看。

service 雖然提供許多組態設定，但是如果你已經知道使用的 image 或是你自己寫的 Dockerfile 原本有寫了，就不用在這邊在寫一次，比如說像等等範例會用到的 wordpress image，他預設就有定義 entrypoint，這時你就不用在這邊重新定義一次。

以下簡介一些 services 底下的組態設定：

build

如果你的 service 要使用 Dockerfile 來建立，你可以透過此設定，指定你 Dockerfile build context 在哪，如果你 build 和 image 兩個設定一起使用，則透過 Dockerfile 建立起來的 image 會被命名成你在 image 設定寫的名稱。需要注意的是，要透過 docker stack 指令部署到 swarm 上時，這設定會失效，因為 docker stack 只接受預先建好的 image。

deploy

只有在 Compose file 版本 3 才能使用。此設定只有在使用 docker stack deploy 指令將應用服務部署到 swarm 上才會有作用，在 docker-compose up、docker-compose down 會被忽略。

depends_on

可以定義 service 之間的相依性，比如說官方範例：
```
version: '2'
services:
  web:
    build: .
    depends_on:
      - db
      - redis
  redis:
    image: redis
  db:
    image: postgres
```
當下 docker-compose up 指令時，db service 和 redis service 會先啟動，然後在啟動 web service。

environment

定義環境變數，注意，如果值是布林，要用單引號包起來，如 'true'、'false'。否則 YML 解析器會把它變成 True 或是 False。以下為官方範例：
```
environment:
  RACK_ENV: development
  SHOW: 'true'
  SESSION_SECRET:

environment:
  - RACK_ENV=development
  - SHOW=true
  - SESSION_SECRET
```
image

指定 container 要從哪個 image 啟動。以下為官方範例：
```
image: redis
image: ubuntu:14.04
image: tutum/influxdb
image: example-registry.com:4000/postgresql
image: a4bc65fd
```
networks

container 要加入哪個網路，這邊的項目會參考到最外層 networks 的設定。

ports

讓你指定要導出的 port，可以是 HOST:CONTAINER，或是只指定 CONTAINER，這時會隨機挑一個 HOST Port 來用。注意，port 指定最好都用字串，因為 YAML 解析器在你挑的 port 小於 60 時會出問題。以下為官方範例：
```
ports:
 - "3000"
 - "3000-3005"
 - "8000:8000"
 - "9090-9091:8080-8081"
 - "49100:22"
 - "127.0.0.1:8001:8001"
 - "127.0.0.1:5000-5010:5000-5010"
 - "6060:6060/udp"
 ```
volumes, volume_driver

設定 container 要使用的 volume，可以是一個路徑或是參考到最外層 volume 的設定，格式為 HOST:CONTAINER，你也可以只寫 CONTAINER，讓 Docker 自動幫你建立一個。參考官方範例：
```
volumes:
  # Just specify a path and let the Engine create a volume
  - /var/lib/mysql

  # Specify an absolute path mapping
  - /opt/data:/var/lib/mysql

  # Path on the host, relative to the Compose file
  - ./cache:/tmp/cache

  # User-relative path
  - ~/configs:/etc/configs/:ro

  # Named volume
  - datavolume:/var/lib/mysql
```
**networks** (top-level)

networks 讓你設定網路。類似 docker network 指令。可以和 servcies 區塊內的 network 搭配使用，例如：
```
version: '3'

services:
  t1:
    image: tomcat:8.5.11-jre8
    networks:
      - test-net
  t2:
    image: tomcat:8.5.11-jre8
    networks:
      - test-net
networks:
  test-net:
```
**volumes** (top-level)

volumes 讓你處理資料共享與資料持久(persist)。類似 docker volume 指令。可以和 servcies 區塊內的 volume 搭配使用，比如說像官方範例：
```
version: "3"

services:
  db:
    image: db
    volumes:
      - data-volume:/var/lib/db
  backup:
    image: backup-service
    volumes:
      - data-volume:/var/lib/backup/data

volumes:
  data-volume:
```
特別要注意的地方是，雖然官方說 Compose file 分成三大組態設定，但是其實最上面還有一個 version 設定，這是一定要加的，不然在執行 docker-compose up 時會一直出現無法理解的錯誤。例如：
```
version: "3"
services:
  foobar:
    image: tomcat:8.5.11-jre8
    networks:
      - foobar-nets
volumes:
  foobar-volume:
networks:
  foobar-nets:
```
更詳細的 Compose file 組態設定介紹建議可以參考官方網站 [Compose file version 3 reference](https://docs.docker.com/compose/compose-file/)。

### Reference
http://blog.maxkit.com.tw/2017/03/docker-compose.html

