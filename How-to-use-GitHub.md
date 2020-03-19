fork from [alecGraves/wiki](https://github.com/alecGraves/wiki/)

### Cloning a repository (with submodules)
A repository or repo contains all of the source code for a specific project.
To download a repository, the most common practice is to clone it.
This command downloads all of the files in the repo to a local directory (folder):

    git clone --recursive <repo URL>

### Using branches
Branches are copies of the repository that are used for developing specific features. 
It allows different parts of a project to be developed without it disrupting the whole project.
Branches are merged into the master branch once work on a specific feature is complete.

To checkout an existing branch, go to a repo directory and run this command

    git checkout <branch name>

To create a new branch, run this command

    git checkout -b <new branch name>

To see what branch you are on, run this command

    git branch

To upload a new branch to GitHub, run this command from the new branch

    git push

### See what GitHub is tracking
Red stuff is untracked, green stuff is tracked. 

    git status

### Track changes to the code
When you are satisfied with a change, begin track it with

    git add <path/to/fileOrDirectory>

Then, run

    git status

To check if your change is being tracked. Green files/directories are tracked, red code is untracked.

### Commit changes to the code
A commit contains a certain change, a message explaining the change, information about when the change occurred, and a unique ID. Please follow [this style guide](http://chris.beams.io/posts/git-commit/) when commiting changes to this repository. 
It will make it easier for others to determine exactly what and why something was changed.

To commit all tracked changes, run

    git commit

This will take you into a text editor. In the first line, make a header for the commit. If the change is complicated and requires more justification, go to a new line and elaborate there. After you are done, save the file.

A commit takes all tracked code and pushes that code into the local repository "head" (the place you are working right now). To upload your changes to GitHub, first make sure your local environment is up to date with the remote(online version):

    git fetch
    git rebase origin/<branch>

Then push your commit to the remote (online version) of the branch you are working on:

    git push

