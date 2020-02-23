# speedrun1-kl25z
Local environment for KL25Z development using Makefile

# Requirements:

https://git-scm.com/
https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads/6-2017-q2-update
https://www.python.org/

(don't forget to add last 2 dependencies to PATH. Specifically for GNU ARM toolchain:

C:\Program Files (x86)\GNU Tools ARM Embedded\6 2017-q2-update\bin)

# Getting started:

```
git clone git@github.com:gubi7g/speedrun1-kl25z.git
cd speedrun1-kl25z
make (WORKS ONLY ON GIT CMD. DON'T ASK ME WHY!?!?!?)
python -m venv env
env/Scripts/activate
pip install -r requirements.txt
```

# To write to board:

```
write.ps1 (Just for Windows PS)
```

# Camera:

```
python camera.py
```

