# 安装git(如果已安装可跳过)

## Windows

### 安装Scoop
终端中运行
```shell
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
Invoke-RestMethod -Uri https://get.scoop.sh | Invoke-Expression
```
等待安装完成

### 安装git
```shell
scoop install git
```
有关git的操作可以参考[这里](https://www.runoob.com/git/git-tutorial.html)

# 克隆代码

```shell
git clone https://github.com/MareDevi/RoboVolley.git
```
如果你的网络不太好：
注意：使用该镜像时你需要一个gitee账号，请注册后进行克隆。（进行克隆操作时可能会让你填写账号密码）

```shell
git clone https://gitee.com/maredevi/RoboVolley.git
```

# 配置git身份

```shell
git config --global user.name '你的用户名' 
git config --global user.email '你的邮箱'
```

# 创建分支(Optional)

``` shell
git checkout -b "分支名" #如 git checkout -b "chassis"
```

# 代码修改完成后提交

强烈建议你在每次提交前都pull一下拉取最新代码
``` shell
git pull
```

提交代码

```shell
git add .
git commit -m "这里是你的修改信息"
git push #提交代码
```