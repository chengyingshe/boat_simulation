# 小船自主导航仿真

一个基于HTML5 Canvas和PathFinding.js的Web前端应用，模拟小船在复杂环境中的自主导航和智能避障。

## 🚢 项目特性

- **激光雷达检测**：100像素检测范围，实时障碍物感知
- **动态避障**：基于A*算法的智能路径规划
- **交互式环境**：拖拽添加障碍物，右键设置目标点
- **实时可视化**：路径、检测范围、导航状态一目了然
- **响应式设计**：现代化UI界面，良好的用户体验

## 🎮 功能演示

### 核心功能

1. **激光雷达传感器**：小船搭载激光雷达，检测半径100像素范围内的障碍物
2. **障碍物管理**：通过鼠标拖拽在水面上添加矩形障碍物
3. **目标设置**：右键点击设置小船的目标位置
4. **自主导航**：点击按钮启动，小船自动规划路径并实时避障
5. **动态重规划**：检测到新障碍物时自动更新路径

### 可视化元素

- 🌊 **蓝色水面**：仿真环境背景
- 🚢 **红色小船**：三角形船只，显示当前方向
- 🎯 **绿色目标点**：导航终点标记
- ⬜ **灰色障碍物**：静态障碍物
- 💛 **黄色路径线**：规划路径可视化
- ⚪ **白色雷达圈**：激光雷达检测范围
- 🔴 **红色高亮**：检测到的障碍物标记

## 🛠️ 技术栈

- **前端渲染**：HTML5 Canvas 2D
- **交互逻辑**：Vanilla JavaScript
- **路径规划**：PathFinding.js (A*算法)
- **部署平台**：Vercel
- **开发工具**：现代浏览器支持

## 📋 使用说明

### 基本操作

1. **添加障碍物**

   - 按住鼠标左键并拖拽
   - 释放鼠标创建矩形障碍物
   - 障碍物会阻挡小船路径
2. **设置目标点**

   - 右键点击画布任意位置
   - 绿色圆点标记目标位置
   - 可重复设置覆盖前一个目标
3. **启动导航**

   - 点击"开始自主导航"按钮
   - 小船开始向目标点移动
   - 黄色线条显示规划路径
4. **观察避障**

   - 白色圆圈显示雷达检测范围
   - 红色高亮显示检测到的障碍物
   - 检测到新障碍物时路径自动重规划

### 控制按钮

- **开始自主导航**：启动小船自动导航
- **重置仿真**：重置所有状态到初始位置
- **清除障碍物**：移除所有障碍物

### 状态指示

- **就绪**：可以开始设置障碍物和目标
- **正在导航**：小船正在执行导航任务
- **导航完成**：小船已到达目标位置

## 🚀 快速开始

### 在线体验

直接访问部署地址即可体验：[Live Demo](https://boat-simulation.vercel.app)

### 本地运行

1. **克隆项目**

   ```bash
   git clone https://github.com/chengyingshe/boat_simulation
   cd boat_simulation
   ```
2. **安装依赖**

   ```bash
   npm install
   ```
3. **启动服务**

   ```bash
   npm start
   ```
4. **访问应用**

   ```
   http://localhost:3000
   ```

## 🌐 部署到Vercel

### 自动部署

1. Fork本仓库到你的GitHub账户
2. 访问 [Vercel](https://vercel.com)
3. 点击"New Project"并选择你的仓库
4. Vercel自动检测配置并部署
5. 获得访问链接

### 手动部署

```bash
# 安装Vercel CLI
npm i -g vercel

# 在项目目录执行
vercel

# 按提示完成配置
```

## 🎯 算法原理

### A*路径规划

- **网格化地图**：将800x600画布划分为40x30网格
- **启发式搜索**：使用曼哈顿距离作为启发函数
- **动态更新**：检测到新障碍物时重新规划路径
- **平滑移动**：网格路径转换为平滑的世界坐标

### 激光雷达模拟

- **圆形检测区域**：以小船为中心的100像素半径
- **障碍物感知**：计算障碍物中心与小船距离
- **实时更新**：每帧检测并更新已知障碍物列表

### 导航控制

- **路径跟随**：沿规划路径移动，动态调整船只角度
- **碰撞避免**：检测路径冲突并重新规划
- **目标到达**：接近目标点时停止导航

## 🔧 自定义配置

### 关键参数

```javascript
const gridSize = 20;        // 网格大小（像素）
const lidarRange = 100;     // 雷达检测范围（像素）
const boatSpeed = 2;        // 小船移动速度（像素/帧）
const canvasSize = {        // 画布尺寸
  width: 800,
  height: 600
};
```

### 可扩展功能

- 添加多个小船
- 实现动态障碍物
- 增加风力影响
- 优化路径平滑度
- 添加物理碰撞检测

## 🐛 问题排查

### 常见问题

1. **路径无法规划**

   - 检查起点和终点是否被障碍物包围
   - 确保目标点在画布范围内
2. **小船不移动**

   - 确认已设置目标点（右键点击）
   - 检查是否点击了"开始自主导航"按钮
3. **障碍物无法添加**

   - 确保拖拽距离足够长（最小10x10像素）
   - 检查鼠标左键是否正常

### 性能优化

- 大量障碍物时可能影响性能
- 建议障碍物数量控制在50个以内
- 复杂路径规划可能需要更多计算时间

## 📄 许可证

本项目采用 MIT 许可证 - 查看 [LICENSE](LICENSE) 文件了解详情

## 🤝 贡献

欢迎提交Issue和Pull Request来改进这个项目！

### 开发指南

1. Fork项目
2. 创建功能分支：`git checkout -b feature/AmazingFeature`
3. 提交更改：`git commit -m 'Add AmazingFeature'`
4. 推送分支：`git push origin feature/AmazingFeature`
5. 提交Pull Request

## 📞 联系方式

如有问题或建议，请通过以下方式联系：

- 提交Issue：[GitHub Issues](https://github.com/chengyingshe/boat_simulation/issues)
- 邮箱：maximeshe@163.com

---

⭐ 如果这个项目对你有帮助，请给个Star支持一下！
