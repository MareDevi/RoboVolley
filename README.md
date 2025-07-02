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

# 克隆代码

```shell
git clone https://github.com/MareDevi/RoboVolley.git
```

# 创建分支(Optional)

``` shell
git checkout -b "分支名" #如 git checkout -b "chassis"
```

# 代码修改完成后提交

```shell
git add .
git commit -m "这里是你的修改信息"
git push #提交代码
```