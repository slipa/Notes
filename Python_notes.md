### Read file line by line using enumerate()
```
filepath = 'Iliad.txt'  
with open(filepath) as fp:  
   for cnt, line in enumerate(fp):
       print("Line {}: {}".format(cnt, line))
```


### Check Object Type

语法：isinstance（object，type）   
作用：来判断一个对象是否是一个已知的类型。  

其第一个参数（object）为对象，第二个参数（type）为类型名(int…)或类型名的一个列表((int,list,float)是一个列表。其返回值为布尔型（True or flase）。  

若对象的类型与参数二的类型相同则返回True。若参数二为一个元组，则若对象类型与元组中类型名之一相同即返回True。  

test

```
if not (isinstance(start_point, list) ) :
        return False
```
[reference](http://blog.csdn.net/wanglei_storage/article/details/52849081)
