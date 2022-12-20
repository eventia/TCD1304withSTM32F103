cls

@echo ON 
@echo *********************************************
@echo *                                           *
@echo *       git pull and git push               *
@echo *                                           *
@echo *********************************************
@echo \n

git pull

git add --all .
git commit -m "HomeNB %date%-[%time:~0,5%]"
git push
