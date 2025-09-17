- Install Git for windows here: <https://git-scm.com/downloads/win>
- Add an ssh key to your github account (makes things easier): 
  - Open command prompt and run this to generate the key:

    ```
    ssh-keygen -t ed25519 -C "github"
    ```
  - Print the key using:

    ```
    type %USERPROFILE%\.ssh\id_ed25519.pub
    ```
  - Copy the full key, it should look like "ssh-ed25519 AAA... github"
  - go to <https://github.com/settings/keys>
  - click "New SSH key" and paste the public key there, give it some title and click "add SSH key"
- Create or open a folder where you want to store the tour robot code. Right click and "Open Git Bash here" (or you can navigate with "cd ..")
- Run this in git bash:

```
git clone git@github.com:arielfayol37/SAGE_ROBOT.git
```

- Type "yes" to connect and enter your password
- Install STM32CubeIDE (Currently: Version: 1.19.0) <https://www.st.com/en/development-tools/stm32cubeide.html>
- Open the SAGE_MCU folder and click on ".project", you might need to set the workspace to the SAGE_ROBOT folder
- Set pins and upload code, when you are done make sure to commit your code

```
git add <filename>
git commit -a
git push
```

... TBD