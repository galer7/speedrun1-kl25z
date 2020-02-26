# speedrun1-kl25z
Local environment for KL25Z development using Makefile

# Requirements:

https://git-scm.com/
https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads/6-2017-q2-update
https://www.python.org/

Don't forget to add last 2 dependencies to PATH.

Specifically for GNU ARM toolchain:

C:\Program Files (x86)\GNU Tools ARM Embedded\6 2017-q2-update\bin

...and python:
C:\Users\user\AppData\Local\Programs\Python\Python38-32

# Getting started:

```
git clone git@github.com:gubi7g/speedrun1-kl25z.git
cd speedrun1-kl25z
make (~~WORKS ONLY ON GIT CMD. DON'T ASK ME WHY!?!?!~~ make works only in Unix based terminals)
```

# Camera:

```
python -m venv env
env/Scripts/activate
pip install -r requirements.txt
python camera.py
```

