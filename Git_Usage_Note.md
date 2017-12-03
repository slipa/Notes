### To remove a directory and everything inside it from the index
```
git rm --cached -r dir
```
The **_--cached_** switch makes git rm operate on the index only and not touch the working copy. The **_-r_** switch makes it recursive.

