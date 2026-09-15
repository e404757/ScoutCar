# Git 常用命令速查表

这份笔记适用于 Windows PowerShell。遇到不确定的情况，先运行 `git status`。

## 1. 查看 Git 信息

```powershell
git --version                  # 查看 Git 版本
git status                     # 查看当前仓库状态
git status --short             # 用精简格式查看状态
git branch --show-current      # 查看当前分支
git remote -v                  # 查看远程仓库地址
```

## 2. 首次配置用户名和邮箱

```powershell
git config --global user.name "你的名字"
git config --global user.email "你的邮箱"
git config --global --list
```

`--global` 表示这项配置对当前电脑上的所有 Git 仓库生效。

## 3. 下载或创建仓库

```powershell
# 第一次从 GitHub 下载项目
cd E:\Projects\Github
git clone https://github.com/用户名/仓库名.git

# 在现有文件夹中创建 Git 仓库
cd E:\Projects\Github\项目名
git init
```

- `git clone`：第一次把远程仓库下载到本地。
- `git pull`：更新一个已经下载到本地的仓库。

## 4. 查看文件修改

```powershell
git status                     # 查看哪些文件发生了变化
git diff                       # 查看尚未暂存的具体修改
git diff --staged              # 查看已经暂存、准备提交的修改
git diff HEAD                  # 查看相对于最近一次提交的全部修改
```

## 5. 暂存并提交修改

```powershell
git add 文件名                 # 暂存一个文件
git add .                      # 暂存当前目录下的所有修改
git commit -m "说明修改内容"   # 把暂存的修改提交到本地仓库
```

推荐流程：

```powershell
git status
git diff
git add 文件名
git diff --staged
git commit -m "说明修改内容"
```

提交说明尽量说清楚“做了什么”，例如：

```powershell
git commit -m "修复车辆列表加载失败问题"
git commit -m "添加项目安装说明"
```

## 6. 获取和上传远程修改

```powershell
git fetch origin               # 下载远程信息，但不修改当前文件
git pull --ff-only             # 安全地获取并合并远程更新
git push                       # 上传当前分支的本地提交
```

第一次上传新分支：

```powershell
git push -u origin 分支名
```

设置 `-u` 后，以后在该分支上通常只需要运行 `git push`。

## 7. 分支操作

```powershell
git branch                     # 查看本地分支
git branch -a                  # 查看本地和远程分支
git switch 分支名              # 切换到已有分支
git switch -c 新分支名         # 创建并切换到新分支
git merge 分支名               # 把指定分支合并到当前分支
git branch -d 分支名           # 删除已合并的本地分支
```

常见开发流程：

```powershell
git switch main
git pull --ff-only
git switch -c feature/功能名称

# 修改文件后
git add .
git commit -m "添加某项功能"
git push -u origin feature/功能名称
```

## 8. 查看提交历史

```powershell
git log
git log --oneline
git log --oneline --graph --decorate --all
git show 提交编号              # 查看某次提交
git show --stat 提交编号       # 查看某次提交改动了哪些文件
```

提交编号通常可以使用 `git log --oneline` 显示的前几位字符。

## 9. 临时保存未完成的修改

```powershell
git stash push -m "临时说明"   # 临时保存当前修改
git stash list                 # 查看临时保存列表
git stash pop                  # 恢复最近一次保存并将其移出列表
git stash apply                # 恢复最近一次保存但保留记录
```

## 10. 撤销操作

先用 `git status` 判断修改处于哪个阶段。

```powershell
# 撤销工作区中某个尚未暂存文件的修改
git restore 文件名

# 将文件移出暂存区，但保留文件修改
git restore --staged 文件名

# 创建一个新提交，撤销指定的旧提交；适合已推送的历史
git revert 提交编号

# 修改最近一次提交说明，或补充刚漏掉的暂存内容
git commit --amend
```

注意：`git restore 文件名` 会丢弃这个文件尚未提交的修改，运行前先确认。

## 11. 远程仓库操作

```powershell
git remote -v
git remote add origin https://github.com/用户名/仓库名.git
git remote set-url origin https://github.com/用户名/新仓库名.git
git fetch --prune               # 更新远程信息并清理失效的远程分支引用
```

## 12. 一次更新多个本地仓库

下面的 PowerShell 命令会更新 `E:\Projects\Github` 下的所有直接子仓库：

```powershell
Get-ChildItem E:\Projects\Github -Directory |
    Where-Object { Test-Path (Join-Path $_.FullName ".git") } |
    ForEach-Object {
        Write-Host "`n正在更新 $($_.Name)" -ForegroundColor Cyan
        git -C $_.FullName pull --ff-only
    }
```

如果某个仓库存在未提交修改或分支冲突，Git 可能会停止更新。进入该仓库后先运行：

```powershell
git status
git diff
```

## 13. 常用日常流程

开始工作：

```powershell
cd E:\Projects\Github\项目名
git status
git pull --ff-only
```

完成工作：

```powershell
git status
git diff
git add 文件名
git diff --staged
git commit -m "说明修改内容"
git push
```

## 14. 初学阶段谨慎使用的命令

以下命令可能删除修改或改写提交历史。没有理解后果前，不要直接运行：

```powershell
git reset --hard
git clean -fd
git push --force
git rebase
```

如果 Git 报错，把下面三条命令的输出保存下来再分析：

```powershell
git status
git branch -vv
git remote -v
```

## 15. Git 代理配置

下面以本地代理端口 `7890` 为例。请根据代理软件显示的实际端口修改。

HTTP/HTTPS 代理：

```powershell
git config --global http.proxy "http://127.0.0.1:7897"
git config --global https.proxy "http://127.0.0.1:7897"
```

SOCKS5 代理：

```powershell
git config --global http.proxy "socks5h://127.0.0.1:7890"
git config --global https.proxy "socks5h://127.0.0.1:7890"
```

`socks5h` 会让域名解析也经过代理。

查看当前代理配置：

```powershell
git config --global --get http.proxy
git config --global --get https.proxy
```

测试 GitHub 连接：

```powershell
git ls-remote https://github.com/e404757/ScoutCar.git
```

如果命令输出提交编号和分支名称，说明 Git 可以连接远程仓库。

取消全局代理：

```powershell
git config --global --unset http.proxy
git config --global --unset https.proxy
```

注意：以上配置适用于 `https://github.com/...` 形式的远程地址。使用 `git@github.com:...` 的 SSH 地址时，需要另外配置 SSH 代理。
