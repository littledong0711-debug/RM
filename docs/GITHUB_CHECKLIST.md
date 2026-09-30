# GitHub提交前检查

[返回首页](../README.md)

## 先确认

- [ ] 运行bash scripts/build_and_test.sh，3项测试通过。
- [ ] package.xml的maintainer目前仍是candidate / candidate@example.com：改成你愿意公开的姓名或昵称和邮箱。
- [ ] package.xml原来写MIT，但仓库没有LICENSE文件。选择授权方式并保持二者一致后再公开；本次未替你授予许可证。
- [ ] 确认考核方允许公开解题源码及test/data中的游戏图像；不确定时先建私有仓库。
- [ ] 不上传游戏二进制、任务书PDF、完整录屏或第三方依赖包。
- [ ] README如实说明AI辅助实现；理解源码后再用于面试讲解，不把测试通过等同于完全自主掌握。

## 哪些应提交

README.md、docs中的公开说明、src、scripts、tools、run_test.sh，以及.gitignore/.gitattributes/.editorconfig/.clang-format。
不要提交build、install、log、diagnostics、docs/local、dist。忽略规则只影响未跟踪文件；若曾提交过这些文件，需另行检查索引。

## 生成干净源码包

```bash
python3 tools/export_source.py
```

Windows也可用python tools/export_source.py。输出dist/armor_aim_source.zip，按白名单收集源码与文档，不会收集本地日志/聊天记录。压缩包生成成功并不代表已上传。
如果使用GitHub网页上传，应先解压，上传其中目录和文件，才能直接浏览代码；不要只把ZIP作为仓库内容。

## 使用Git上传

在本项目根目录执行，而不是上一级工作目录。以下命令需你确认上述清单后自行执行：

```bash
git init
git add .
git status --short
git diff --cached --stat
```

仔细检查待提交文件后再commit。创建远程仓库并添加自己的remote地址，再push。
本次整理未初始化仓库、未创建远程仓库、未提交或上传，也不包含登录凭据。
