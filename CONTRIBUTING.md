# 团队协作指南

## 权限与首次准备

队长拥有仓库管理权限，队友以协作者身份获得代码写入权限。每个人使用自己的 GitHub 账户，接受仓库邀请后再操作。

```bash
git clone https://github.com/pfeffermominee690-afk/smart-car.git
cd smart-car
git config user.name "你的名字"
git config user.email "你的 GitHub 提交邮箱"
```

## 每个功能的提交流程

以下分支名和文件路径是示例，请替换成自己的模块。

```bash
git switch main
git pull --ff-only origin main
git switch -c feat/lane-detection
# 修改文件，执行模块测试
git add src/lane_follow/python/lane_follow/vision.py src/lane_follow/tests/test_lane.py
git commit -m "feat(vision): add lane detection"
git push -u origin feat/lane-detection
```

打开 GitHub 仓库的 **Pull requests → New pull request**，选择 `base: main` 和自己的功能分支。按模板写明改动、接口影响及测试证据。草稿阶段可使用 Draft PR；准备好再请求队长审核。

提交后按反馈修改并推送到同一个功能分支。新提交会使旧批准失效，需要重新审核。队友互相评审有帮助，但不能替代指定负责人的批准。

## 队长如何确认

1. 打开 PR 的 **Files changed** 检查代码及配置。
2. 核对模块测试、接口兼容性和必要的实车验证记录。
3. 点击 **Review changes → Approve → Submit review**；有问题时选择 **Request changes**。
4. 满足合并条件后点击 **Squash and merge**。

聊天里的口头同意、普通评论或点赞不等于 GitHub 的 Approve。批准后，拥有写入权限的协作者也可能执行合并；关键约束是合并前必须获得队长批准。

GitHub 不允许批准自己发起的 PR。如果队长是唯一 Code Owner，队长自己发起的 PR 需要采用明确的管理员处理方式，或之后单独设计可信的备用审核流程；不要随意增加 Code Owner，否则该人也能批准其他队友的代码。

## 模块与测试约定

- PR 描述输入/输出、ROS 话题或接口、坐标系、单位及配置默认值。
- 标定参数注明车辆、日期和条件，区分原始控制值与物理单位。
- 算法变更提供离线测试或回放结果；实车结论说明测试范围。
- 涉及底盘控制时说明停车、输入超时及命令冲突处理方式。
- 日志、录像、数据集和模型权重保存到团队约定的外部存储，在 PR 中链接相关证据。
- 不提交设备密码、令牌、私钥或个人环境配置；可用不含凭据的 `.example` 文件说明配置方法。

## 合并后继续开发

```bash
git switch main
git pull --ff-only origin main
git switch -c feat/next-feature
```

每次从最新主分支开始新任务。稳定比赛版本由队长命名并打 tag，部署记录应包含 commit SHA 或 tag，方便回滚和复现。
