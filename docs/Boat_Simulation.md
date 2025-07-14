# 小船自主导航仿真程序开发文档

## 项目概述
本项目开发一个基于Web前端的小船自主导航仿真程序，满足以下功能要求：
1. **激光雷达传感器**：小船搭载激光雷达，检测半径为2m（模拟为100像素）的圆形范围内障碍物，并可视化检测范围。
2. **障碍物添加**：用户通过拖拽在水面上添加立方体障碍物（2D中为矩形）。
3. **终点设置**：用户通过右键点击指定小船的终点位置。
4. **自主导航**：点击“开始自主导航”按钮后，小船自动规划路径，实时可视化路径（黄色线条），并在激光雷达检测到障碍物后重新规划路径。

程序使用HTML5 Canvas绘制2D仿真环境，包括蓝色水面、红色三角形小船、灰色矩形障碍物、绿色终点标记、黄色路径线和白色激光雷达范围圈。路径规划采用A*算法，仅考虑激光雷达检测到的障碍物。

## 技术栈
- **HTML5 Canvas**：用于2D图形渲染。
- **JavaScript**：处理用户交互、路径规划、动画和逻辑。
- **PathFinding.js**：实现A*路径规划，通过CDN引入。
- **Vercel**：用于自动部署Web应用。
- **浏览器**：支持HTML5的现代浏览器（如Chrome、Firefox）。

## 实现步骤

### 1. 项目结构
为支持Vercel部署，项目文件结构如下：

```
boat-simulation/
├── index.html
├── package.json
└── README.md
```

### 2. 代码实现
以下是核心代码，保存为`index.html`：

```html
<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>小船自主导航仿真</title>
    <style>
        body { margin: 0; padding: 10px; }
        canvas { border: 1px solid black; }
    </style>
</head>
<body>
    <canvas id="simulationCanvas" width="800" height="600"></canvas>
    <button id="startButton">开始自主导航</button>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/pathfinding/0.4.18/pathfinding-browser.min.js"></script>
    <script>
        const canvas = document.getElementById('simulationCanvas');
        const ctx = canvas.getContext('2d');
        const width = canvas.width;
        const height = canvas.height;
        const gridSize = 20;
        const numCols = Math.floor(width / gridSize);
        const numRows = Math.floor(height / gridSize);
        const lidarRange = 100;

        class Obstacle {
            constructor(x, y, width, height) {
                this.x = x;
                this.y = y;
                this.width = width;
                this.height = height;
            }

            contains(pointX, pointY) {
                return pointX >= this.x && pointX <= this.x + this.width &&
                       pointY >= this.y && pointY <= this.y + this.height;
            }

            getOccupiedCells() {
                const cells = [];
                const startCol = Math.floor(this.x / gridSize);
                const endCol = Math.floor((this.x + this.width) / gridSize);
                const startRow = Math.floor(this.y / gridSize);
                const endRow = Math.floor((this.y + this.height / gridSize));
                for (let row = startRow; row <= endRow; row++) {
                    for (let col = startCol; col <= endCol; col++) {
                        if (row >= 0 && row < numRows && col >= 0 && col < numCols) {
                            cells.push({col, row});
                        }
                    }
                }
                return cells;
            }
        }

        class Boat {
            constructor(x, y) {
                this.x = x;
                this.y = y;
                this.angle = 0;
                this.speed = 2;
                this.path = [];
                this.gridPath = [];
                this.isNavigating = false;
            }

            move() {
                if (this.path.length > 0) {
                    const nextPoint = this.path[0];
384                    const dx = nextPoint.x - this.x;
                    const dy = nextPoint.y - this.y;
                    const distance = Math.sqrt(dx * dx + dy * dy);
                    if (distance < this.speed) {
                        this.x = nextPoint.x;
                        this.y = nextPoint.y;
                        this.path.shift();
                        this.gridPath.shift();
                    } else {
                        const angle = Math.atan2(dy, dx);
                        this.x += this.speed * Math.cos(angle);
                        this.y += this.speed * Math.sin(angle);
                        this.angle = angle;
                    }
                } else {
                    this.isNavigating = false;
                }
            }

            detectObstacles(allObstacles) {
                const detected = [];
                for (const obst of allObstacles) {
                    const dist = Math.hypot(obst.x - this.x, obst.y - this.y);
                    if (dist < lidarRange) {
                        detected.push(obst);
                    }
                }
                return detected;
            }
        }

        let allObstacles = [];
        let knownObstacles = [];
        let destination = null;
        let boat = new Boat(100, 100);
        let isDragging = false;
        let startX, startY;
        const grid = new PF.Grid(numCols, numRows);

        function worldToGrid(x, y) {
            return {col: Math.floor(x / gridSize), row: Math.floor(y / gridSize)};
        }

        function gridToWorld(col, row) {
            return {x: col * gridSize + gridSize / 2, y: row * gridSize + gridSize / 2};
        }

        function updateGrid(grid, knownObstacles) {
            for (let row = 0; row < numRows; row++) {
                for (let col = 0; col < numCols; col++) {
                    grid.setWalkableAt(col, row, true);
                }
            }
            for (const obst of knownObstacles) {
                const cells = obst.getOccupiedCells();
                for (const cell of cells) {
                    grid.setWalkableAt(cell.col, cell.row, false);
                }
            }
        }

        function planPath(boat, destination, grid, knownObstacles) {
            updateGrid(grid, knownObstacles);
            const start = worldToGrid(boat.x, boat.y);
            const end = worldToGrid(destination.x, destination.y);
            const finder = new PF.AStarFinder();
            const gridPath = finder.findPath(start.col, start.row, end.col, end.row, grid.clone());
            const worldPath = gridPath.map(p => gridToWorld(p[0], p[1]));
            return {gridPath, worldPath};
        }

        canvas.addEventListener('mousedown', (e) => {
            const rect = canvas.getBoundingClientRect();
            const mouseX = e.clientX - rect.left;
            const mouseY = e.clientY - rect.top;
            if (e.button === 0) {
                isDragging = true;
                startX = mouseX;
                startY = mouseY;
            } else if (e.button === 2) {
                destination = {x: mouseX, y: mouseY};
                e.preventDefault();
            }
        });

        canvas.addEventListener('mouseup', (e) => {
            if (isDragging) {
                isDragging = false;
                const rect = canvas.getBoundingClientRect();
                const mouseX = e.clientX - rect.left;
                const mouseY = e.clientY - rect.top;
                const width = mouseX - startX;
                const height = mouseY - startY;
                const obstWidth = Math.abs(width);
                const obstHeight = Math.abs(height);
                const obstX = width > 0 ? startX : startX + width;
                const obstY = height > 0 ? startY : startY + height;
                allObstacles.push(new Obstacle(obstX, obstY, obstWidth, obstHeight));
            }
        });

        canvas.addEventListener('contextmenu', (e) => e.preventDefault());

        document.getElementById('startButton').addEventListener('click', () => {
            if (destination) {
                boat.isNavigating = true;
                const plan = planPath(boat, destination, grid, knownObstacles);
                boat.gridPath = plan.gridPath;
                boat.path = plan.worldPath;
            }
        });

        function animate() {
            ctx.clearRect(0, 0, width, height);
            ctx.fillStyle = 'blue';
            ctx.fillRect(0, 0, width, height);

            ctx.fillStyle = 'gray';
            for (const obst of allObstacles) {
                ctx.fillRect(obst.x, obst.y, obst.width, obst.height);
            }

            if (destination) {
                ctx.fillStyle = 'green';
                ctx.beginPath();
                ctx.arc(destination.x, destination.y, 5, 0, 2 * Math.PI);
                ctx.fill();
            }

            ctx.fillStyle = 'red';
            ctx.save();
            ctx.translate(boat.x, boat.y);
            ctx.rotate(boat.angle);
            ctx.beginPath();
            ctx.moveTo(10, 0);
            ctx.lineTo(-10, 5);
            ctx.lineTo(-10, -5);
            ctx.closePath();
            ctx.fill();
            ctx.restore();

            if (boat.path.length > 0) {
                ctx.strokeStyle = 'yellow';
                ctx.beginPath();
                ctx.moveTo(boat.x, boat.y);
                for (const p of boat.path) {
                    ctx.lineTo(p.x, p.y);
                }
                ctx.stroke();
            }

            ctx.strokeStyle = 'white';
            ctx.beginPath();
            ctx.arc(boat.x, boat.y, lidarRange, 0, 2 * Math.PI);
            ctx.stroke();

            if (boat.isNavigating) {
                const detected = boat.detectObstacles(allObstacles);
                const newKnown = detected.filter(obst => !knownObstacles.includes(obst));
                knownObstacles = knownObstacles.concat(newKnown);
                updateGrid(grid, knownObstacles);
                if (boat.gridPath.some(p => !grid.nodes[p[0] + p[1] * grid.width].walkable)) {
                    const plan = planPath(boat, destination, grid, knownObstacles);
                    boat.gridPath = plan.gridPath;
                    boat.path = plan.worldPath;
                }
                boat.move();
            }

            requestAnimationFrame(animate);
        }

        animate();
    </script>
</body>
</html>
```

### 3. 部署到Vercel
Vercel支持静态Web应用的自动部署，以下是步骤：

#### 3.1 创建`package.json`
为Vercel识别项目，添加以下`package.json`：

```json
{
  "name": "boat-simulation",
  "version": "1.0.0",
  "description": "小船自主导航仿真程序",
  "scripts": {
    "start": "serve -s ."
  },
  "dependencies": {},
  "devDependencies": {
    "serve": "^14.2.0"
  }
}
```

#### 3.2 创建`README.md`
提供项目说明：

```markdown
# 小船自主导航仿真

一个基于HTML5 Canvas和PathFinding.js的Web前端应用，模拟小船自主导航和避障。

## 功能
- 激光雷达检测（100像素范围）
- 拖拽添加障碍物
- 右键设置终点
- 自动路径规划和实时可视化

## 部署
1. 克隆仓库
2. 运行 `npm install`
3. 运行 `npm start` 本地预览
4. 部署到Vercel
```

#### 3.3 部署步骤
1. **初始化Git仓库**：
   ```bash
   git init
   git add .
   git commit -m "Initial commit"
   ```

2. **推送到GitHub**：
   - 创建GitHub仓库（如`boat-simulation`）。
   - 执行：
     ```bash
     git remote add origin <your-repo-url>
     git push -u origin main
     ```

3. **连接Vercel**：
   - 登录[Vercel](https://vercel.com)。
   - 点击“New Project”，选择“Import Git Repository”。
   - 选择你的GitHub仓库，点击“Import”。
   - Vercel自动检测`package.json`，无需额外配置。
   - 点击“Deploy”，Vercel将自动构建并部署应用。

4. **访问应用**：
   - 部署完成后，Vercel提供一个URL（如`https://boat-simulation.vercel.app`）。
   - 打开URL即可运行仿真程序。

#### 3.4 自动部署
- 每次推送到GitHub的`main`分支，Vercel会自动重新部署。
- 配置GitHub Actions（可选）以进一步自动化流程。

### 4. 功能实现
#### 4.1 激光雷达可视化
- 绘制白色圆圈（半径100像素）表示激光雷达范围。
- 检测小船中心与障碍物中心的距离，小于100像素时标记为检测到。

#### 4.2 障碍物添加
- 鼠标左键拖拽创建矩形障碍物，存储在`allObstacles`数组。
- 障碍物占据网格单元，影响路径规划。

#### 4.3 终点设置
- 右键点击设置终点，显示为绿色圆点。
- 存储在`destination`变量，触发路径规划。

#### 4.4 自主导航
- 使用PathFinding.js的A*算法在40x30网格上规划路径。
- 路径以黄色线条实时绘制。
- 激光雷达检测到新障碍物时，更新`knownObstacles`，重新规划路径。

#### 4.5 动画循环
- 使用`requestAnimationFrame`实现平滑动画。
- 每帧清空画布，绘制水面、障碍物、终点、小船、路径和雷达范围。
- 更新小船位置，检查路径有效性。

## 使用说明
1. **添加障碍物**：按住鼠标左键拖拽，释放后生成矩形障碍物。
2. **设置终点**：右键点击设置绿色终点标记。
3. **启动导航**：点击“开始自主导航”按钮，小船向终点移动，路径显示为黄色线条。
4. **激光雷达**：白色圆圈表示检测范围，检测到障碍物时更新路径。

## 注意事项
- **网格化**：路径规划基于20x20像素网格，小船移动为连续路径。
- **性能优化**：仅在路径被新障碍物阻挡时重新规划。
- **Vercel部署**：确保`index.html`为入口文件，CDN依赖无需本地存储。
- **扩展性**：可添加动态水面效果或更复杂的雷达扫描线。

## 参考资源
- [PathFinding.js](https://qiao.github.io/PathFinding.js/visual/)
- [HTML5 Canvas](https://developer.mozilla.org/zh-CN/docs/Web/API/Canvas_API)
- [Vercel Documentation](https://vercel.com/docs)
- [MDN requestAnimationFrame](https://developer.mozilla.org/zh-CN/docs/Web/API/Window/requestAnimationFrame)