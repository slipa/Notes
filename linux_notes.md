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


### Fix failure on mounting windows disk
```
sudo ntfsfix /dev/sda2
```


### 壓縮與解壓縮 使用 tar

選項與參數：
-c  ：建立打包檔案，可搭配 -v 來察看過程中被打包的檔名(filename)  
-t  ：察看打包檔案的內容含有哪些檔名，重點在察看『檔名』就是了  
-x  ：解打包或解壓縮的功能，可以搭配 -C (大寫) 在特定目錄解開，特別留意的是， -c, -t, -x 不可同時出現在一串指令列中。  
-z  ：透過 gzip  的支援進行壓縮/解壓縮：此時檔名最好為 *.tar.gz  
-j  ：透過 bzip2 的支援進行壓縮/解壓縮：此時檔名最好為 *.tar.bz2  
-J  ：透過 xz    的支援進行壓縮/解壓縮：此時檔名最好為 *.tar.xz  
      特別留意， -z, -j, -J 不可以同時出現在一串指令列中  
-v  ：在壓縮/解壓縮的過程中，將正在處理的檔名顯示出來！  
-f filename：-f 後面要立刻接要被處理的檔名！建議 -f 單獨寫一個選項囉！(比較不會忘記)  
-C 目錄    ：這個選項用在解壓縮，若要在特定目錄解壓縮，可以使用這個選項。  


-p(小寫) ：保留備份資料的原本權限與屬性，常用於備份(-c)重要的設定檔  
-P(大寫) ：保留絕對路徑，亦即允許備份資料中含有根目錄存在之意；  

壓　縮：
```
tar -zcv -f filename.tar.gz 要被壓縮的檔案或目錄名稱  
```
查　詢：
```
tar -ztv -f filename.tar.gz  
```
解壓縮：
```
tar -zxv -f filename.tar.gz -C 欲解壓縮的目錄  
```

### 在 Python 中設定GPIO.BOARD與GPIO.BCM有何不同
```
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
```
或者
```
GPIO.setmode(GPIO.BCM)
```
這個GPIO.BOARD 選項是指定在電路版上接脚的號碼  
這個GPIO.BCM 選項是指定GPIO後面的號碼


### How to use systemd to start a service

[reference](https://www.raspberrypi.org/documentation/linux/usage/systemd.md)


### 由Command Line Interface 查看圖檔屬性

```
$ file test01.png
$ test01.png: PNG image data, 568 x 304, 8-bit/color RGB, non-interlaced
```

### 更改圖檔屬性

先安裝imagemagick
```
$ sudo apt install imagemagick
```
```
$ convert <img_in> -set colorspace Gray -separate -average <img_out>
$ convert input.png -alpha off output.png​
```

