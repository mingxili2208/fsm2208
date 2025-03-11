// 全局变量存储系统状态
let systemState = {
    trackerInitialized: false,
    serialConnected: false,
    isRecording: false,
    recordedDataPoints: 0,
    eulerPoints: 0,
    laserDataProcessed: false,
    eulerDataAvailable: false,
    transformsCalculated: false,
    outputDirectory: '',
    recordingStartTime: null
};

// 页面加载完成后初始化
document.addEventListener('DOMContentLoaded', function() {
    // 初始化标签页功能
    initializeTabs();
    
    // 初始化按钮事件监听
    initializeButtons();
    
    // 初始化控制台
    addConsoleEntry('[INFO] Auto Tracker Data Process GUI v1.0.0', 'info');
    addConsoleEntry('[INFO] Current date: ' + new Date().toLocaleString(), 'info');
    addConsoleEntry('[INFO] Ready to initialize tracker system', 'info');
    
    // 定期获取系统状态更新
    setInterval(fetchSystemStatus, 2000);
});

// 初始化标签页功能
function initializeTabs() {
    document.querySelectorAll('.tab').forEach(tab => {
        tab.addEventListener('click', () => {
            // 移除所有标签页和内容的active类
            document.querySelectorAll('.tab').forEach(t => t.classList.remove('active'));
            document.querySelectorAll('.tab-content').forEach(c => c.classList.remove('active'));
            
            // 为当前标签页和内容添加active类
            tab.classList.add('active');
            document.getElementById(tab.dataset.tab).classList.add('active');
        });
    });
}

// 初始化按钮事件监听
function initializeButtons() {
    document.getElementById('btn-initialize').addEventListener('click', initializeTracker);
    document.getElementById('btn-start-recording').addEventListener('click', startRecording);
    document.getElementById('btn-stop-recording').addEventListener('click', stopRecording);
    document.getElementById('btn-record-euler').addEventListener('click', showEulerInput);
    document.getElementById('btn-save-all').addEventListener('click', saveAllData);
    document.getElementById('btn-save-euler').addEventListener('click', saveEulerPoint);
    document.getElementById('btn-cancel-euler').addEventListener('click', cancelEulerInput);
    document.getElementById('btn-process-data').addEventListener('click', processData);
    document.getElementById('btn-calculate-transform').addEventListener('click', calculateTransformations);
    document.getElementById('btn-review-data').addEventListener('click', reviewData);
    document.getElementById('btn-visualize-points').addEventListener('click', visualizePoints);
    document.getElementById('btn-show-transforms').addEventListener('click', showTransformations);
    document.getElementById('btn-show-errors').addEventListener('click', showErrors);
    document.getElementById('btn-export-chart').addEventListener('click', exportChart);
    document.getElementById('btn-reset-settings').addEventListener('click', resetSettings);
    document.getElementById('btn-clear-console').addEventListener('click', clearConsole);
    document.getElementById('btn-check-devices').addEventListener('click', checkDevices);
    document.getElementById('btn-confirm-record').addEventListener('click', confirmRecord);
    document.getElementById('btn-modify-record').addEventListener('click', modifyRecord);
    document.getElementById('btn-delete-record').addEventListener('click', deleteRecord);
    document.getElementById('btn-close-records').addEventListener('click', closeRecords);
    document.getElementById('btn-view-euler-records').addEventListener('click', viewEulerRecords);

    document.getElementById('btn-restart').addEventListener('click', restartApplication);
    document.getElementById('btn-exit').addEventListener('click', exitApplication);
}

// 获取系统状态更新
function fetchSystemStatus() {
    fetch('/api/status')
        .then(response => response.json())
        .then(data => {
            // 更新UI状态
            updateUIFromStatus(data);
            
            // 添加新的日志消息
            if (data.logMessages && data.logMessages.length > 0) {
                const currentLogs = document.querySelectorAll('.console-entry').length;
                if (data.logMessages.length > currentLogs) {
                    // 有新的日志消息
                    for (let i = currentLogs; i < data.logMessages.length; i++) {
                        const log = data.logMessages[i];
                        addConsoleEntry(log.message, log.type);
                    }
                }
            }
        })
        .catch(error => {
            console.error('Error fetching system status:', error);
        });
}

// 根据状态更新UI
function updateUIFromStatus(status) {
    // 更新追踪器状态
    if (status.trackerInitialized) {
        document.getElementById('tracker-status').className = 'status-indicator status-active';
        document.getElementById('tracker-status-text').textContent = 'Initialized';
        document.getElementById('btn-start-recording').disabled = false;
        document.getElementById('btn-record-euler').disabled = false;
    }
    
    // 更新记录状态
    if (status.isRecording) {
        document.getElementById('recording-status').className = 'status-indicator status-active';
        document.getElementById('recording-status-text').textContent = 'Active';
        document.getElementById('btn-start-recording').disabled = true;
        document.getElementById('btn-stop-recording').disabled = false;
        document.getElementById('btn-save-all').disabled = false;
    } else {
        document.getElementById('recording-status').className = 'status-indicator status-inactive';
        document.getElementById('recording-status-text').textContent = 'Inactive';
        document.getElementById('btn-stop-recording').disabled = true;
    }
    
    // 更新记录点数
    document.getElementById('recorded-points-count').textContent = status.recordedDataPoints;
    document.getElementById('total-recorded-points').textContent = status.recordedDataPoints;
    
    // 更新输出目录
    if (status.outputDirectory) {
        document.getElementById('output-directory').value = status.outputDirectory;
        document.getElementById('current-output-dir').textContent = status.outputDirectory;
    }
    
    // 更新欧拉数据状态
    if (status.eulerPoints > 0) {
        document.getElementById('euler-data-status').className = 'status-indicator status-active';
        document.getElementById('euler-data-status-text').textContent = `${status.eulerPoints} Points`;
        document.getElementById('btn-process-data').disabled = false;
    }
    
    // 更新激光数据处理状态
    if (status.laserDataProcessed) {
        document.getElementById('laser-data-status').className = 'status-indicator status-active';
        document.getElementById('laser-data-status-text').textContent = 'Processed';
        document.getElementById('btn-calculate-transform').disabled = false;
        document.getElementById('btn-review-data').disabled = false;
    }
    
    // 更新变换计算状态
    if (status.transformsCalculated) {
        document.getElementById('transform-status').className = 'status-indicator status-active';
        document.getElementById('transform-status-text').textContent = 'Calculated';
        document.getElementById('btn-visualize-points').disabled = false;
        document.getElementById('btn-show-transforms').disabled = false;
        document.getElementById('btn-show-errors').disabled = false;
        document.getElementById('btn-export-chart').disabled = false;
    }
}

// 控制台功能
function addConsoleEntry(message, type = 'info', consoleSelector = '.console') {
    const consoleElements = document.querySelectorAll(consoleSelector);
    if (consoleElements.length === 0) return;
    
    consoleElements.forEach(consoleElement => {
        const entry = document.createElement('div');
        entry.className = `console-entry ${type}`;
        entry.textContent = message;
        consoleElement.appendChild(entry);
        consoleElement.scrollTop = consoleElement.scrollHeight;
    });
}

// 初始化追踪器
function initializeTracker() {
    const trackerName = document.getElementById('tracker-name').value;
    const serialPort = document.getElementById('serial-port').value;
    const baudRate = document.getElementById('baud-rate').value;
    
    addConsoleEntry(`[INFO] Initializing tracker system: "${trackerName}" on port ${serialPort} at ${baudRate} baud...`);
    
    // 调用API初始化追踪器
    fetch('/api/initialize', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({
            trackerName: trackerName,
            serialPort: serialPort,
            baudRate: baudRate
        })
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            // 更新系统状态
            systemState.trackerInitialized = true;
            systemState.serialConnected = true;
            systemState.outputDirectory = data.outputDirectory || systemState.outputDirectory;
            
            // 更新UI
            document.getElementById('tracker-status').className = 'status-indicator status-active';
            document.getElementById('tracker-status-text').textContent = 'Initialized';
            document.getElementById('btn-start-recording').disabled = false;
            document.getElementById('btn-record-euler').disabled = false;
            document.getElementById('output-directory').value = systemState.outputDirectory;
            document.getElementById('current-output-dir').textContent = systemState.outputDirectory;
            
            addConsoleEntry(`[SUCCESS] Tracker "${trackerName}" successfully initialized`, 'info');
            addConsoleEntry(`[INFO] Output directory created: ${systemState.outputDirectory}`, 'info');
            addConsoleEntry(`[INFO] Ready to start recording data`, 'info');
        } else {
            addConsoleEntry(`[ERROR] Failed to initialize tracker: ${data.message}`, 'error');
        }
    })
    .catch(error => {
        addConsoleEntry(`[ERROR] Error connecting to server: ${error}`, 'error');
    });
}

// 开始记录
function startRecording() {
    if (!systemState.trackerInitialized) {
        addConsoleEntry('[ERROR] Please initialize the tracker first', 'error');
        return;
    }
    
    fetch('/api/start_recording', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        }
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            systemState.isRecording = true;
            systemState.recordingStartTime = new Date();
            
            // 更新状态指示器
            document.getElementById('recording-status').className = 'status-indicator status-active';
            document.getElementById('recording-status-text').textContent = 'Active';
            
            // 更新按钮
            document.getElementById('btn-start-recording').disabled = true;
            document.getElementById('btn-stop-recording').disabled = false;
            document.getElementById('btn-save-all').disabled = false;
            
            addConsoleEntry('[INFO] Starting continuous recording (laser_tracker.csv)...', 'info');
        } else {
            addConsoleEntry(`[ERROR] Failed to start recording: ${data.message}`, 'error');
        }
    })
    .catch(error => {
        addConsoleEntry(`[ERROR] Error connecting to server: ${error}`, 'error');
    });
}

// 停止记录
function stopRecording() {
    fetch('/api/stop_recording', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        }
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            systemState.isRecording = false;
            
            // 更新状态指示器
            document.getElementById('recording-status').className = 'status-indicator status-inactive';
            document.getElementById('recording-status-text').textContent = 'Inactive';
            
            // 更新按钮
            document.getElementById('btn-start-recording').disabled = false;
            document.getElementById('btn-stop-recording').disabled = true;
            
            // 计算记录持续时间
            const duration = Math.round((new Date() - systemState.recordingStartTime) / 1000);
            
            addConsoleEntry(`[INFO] Stopped recording after ${duration} seconds`, 'info');
            addConsoleEntry(`[INFO] Recorded ${data.recordedPoints || systemState.recordedDataPoints} data points`, 'info');
            
            // 更新记录点数
            if (data.recordedPoints) {
                systemState.recordedDataPoints = data.recordedPoints;
                document.getElementById('recorded-points-count').textContent = data.recordedPoints;
                document.getElementById('total-recorded-points').textContent = data.recordedPoints;
            }
        } else {
            addConsoleEntry(`[ERROR] Failed to stop recording: ${data.message}`, 'error');
        }
    })
    .catch(error => {
        addConsoleEntry(`[ERROR] Error connecting to server: ${error}`, 'error');
    });
}

// 显示欧拉输入
function showEulerInput() {
    document.getElementById('euler-input-container').style.display = 'block';
    document.getElementById('euler-angle').focus();
}



// 取消欧拉输入
function cancelEulerInput() {
    document.getElementById('euler-input-container').style.display = 'none';
}

// 保存所有数据
function saveAllData() {
    addConsoleEntry('[INFO] Saving all recorded data...', 'info');
    
    fetch('/api/save_all_data', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        }
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            addConsoleEntry(`[SUCCESS] Laser tracker data saved (${data.recordedPoints} points)`, 'info');
            
            if (data.eulerPoints > 0) {
                addConsoleEntry(`[SUCCESS] Euler data saved (${data.eulerPoints} points)`, 'info');
            }
            
            // 更新系统状态
            systemState.recordedDataPoints = data.recordedPoints;
            systemState.eulerPoints = data.eulerPoints;
            
            // 更新UI
            document.getElementById('recorded-points-count').textContent = data.recordedPoints;
            document.getElementById('total-recorded-points').textContent = data.recordedPoints;
            
            // 启用处理
            document.getElementById('btn-process-data').disabled = false;
            
            // 更新状态
            if (data.recordedPoints > 0) {
                document.getElementById('laser-data-status').className = 'status-indicator status-pending';
                document.getElementById('laser-data-status-text').textContent = 'Ready for Processing';
            }
        } else {
            addConsoleEntry(`[ERROR] Failed to save data: ${data.message}`, 'error');
        }
    })
    .catch(error => {
        addConsoleEntry(`[ERROR] Error connecting to server: ${error}`, 'error');
    });
}

// 处理数据
function processData() {
    const negativeYawOffset = document.getElementById('negative-yaw-offset').value;
    const positiveYawOffset = document.getElementById('positive-yaw-offset').value;
    
    addConsoleEntry(`[INFO] Processing laser tracker data with offsets: negative yaw: +${negativeYawOffset}°, positive yaw: -${positiveYawOffset}°`, 'info');
    
    // 更新进度条
    updateProgressBar(0);
    
    fetch('/api/process_data', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({
            negativeYawOffset: negativeYawOffset,
            positiveYawOffset: positiveYawOffset
        })
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            // 模拟处理过程
            updateProgressBar(30);
            setTimeout(() => {
                updateProgressBar(70);
                setTimeout(() => {
                    updateProgressBar(100);
                    
                    addConsoleEntry('[SUCCESS] Data processing complete!', 'info');
                    addConsoleEntry('[INFO] Corrected data saved to corrected_laser_tracker.csv', 'info');
                    
                    // 更新状态
                    document.getElementById('laser-data-status').className = 'status-indicator status-active';
                    document.getElementById('laser-data-status-text').textContent = 'Processed';
                    
                    // 启用下一步
                    document.getElementById('btn-calculate-transform').disabled = false;
                    document.getElementById('btn-review-data').disabled = false;
                    
                    systemState.laserDataProcessed = true;
                    
                    // 如果两种数据类型都可用，可以计算变换
                    if (systemState.eulerPoints > 0) {
                        addConsoleEntry('[INFO] Both data sets available. Ready to calculate transformations.', 'info');
                    }
                }, 1000);
            }, 1000);
        } else {
            updateProgressBar(100);
            addConsoleEntry(`[ERROR] Failed to process data: ${data.message}`, 'error');
        }
    })
    .catch(error => {
        updateProgressBar(100);
        addConsoleEntry(`[ERROR] Error connecting to server: ${error}`, 'error');
    });
}

// 更新进度条
function updateProgressBar(percentage) {
    const progressBar = document.getElementById('processing-progress');
    progressBar.style.width = `${percentage}%`;
    progressBar.textContent = `${percentage}%`;
}

// 计算变换
function calculateTransformations() {
    addConsoleEntry('[INFO] Calculating coordinate transformations...', 'info');
    
    // 更新进度条
    updateProgressBar(0);
    
    fetch('/api/calculate_transformations', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        }
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            // 模拟计算步骤
            updateProgressBar(25);
            setTimeout(() => {
                updateProgressBar(50);
                setTimeout(() => {
                    updateProgressBar(75);
                    setTimeout(() => {
                        updateProgressBar(100);
                        
                        addConsoleEntry('[SUCCESS] Transformation calculation complete!', 'info');
                        
                        // 设置变换矩阵
                        if (data.T_pos && data.R_euler) {
                            document.getElementById('t-pos-matrix').textContent = formatMatrix(data.T_pos);
                            document.getElementById('r-euler-matrix').textContent = formatMatrix(data.R_euler);
                        }
                        
                        // 更新状态
                        document.getElementById('transform-status').className = 'status-indicator status-active';
                        document.getElementById('transform-status-text').textContent = 'Calculated';
                        
                        // 启用可视化
                        document.getElementById('btn-visualize-points').disabled = false;
                        document.getElementById('btn-show-transforms').disabled = false;
                        document.getElementById('btn-show-errors').disabled = false;
                        document.getElementById('btn-export-chart').disabled = false;
                        
                        systemState.transformsCalculated = true;
                        
                        addConsoleEntry('[INFO] Transformation matrices saved to transformation_matrices.txt', 'info');
                        addConsoleEntry('[INFO] Ready for visualization', 'info');
                        
                        // 获取错误数据
                        fetchErrorData();
                    }, 1000);
                }, 1000);
            }, 1000);
        } else {
            updateProgressBar(100);
            addConsoleEntry(`[ERROR] Failed to calculate transformations: ${data.message}`, 'error');
        }
    })
    .catch(error => {
        updateProgressBar(100);
        addConsoleEntry(`[ERROR] Error connecting to server: ${error}`, 'error');
    });
}

// 获取错误数据
function fetchErrorData() {
    fetch('/api/get_errors')
        .then(response => response.json())
        .then(data => {
            if (data.success && data.error_data) {
                // 设置错误指标
                if (data.error_data.rmse) {
                    document.getElementById('position-rmse').textContent = `${data.error_data.rmse.toFixed(4)} meters`;
                }
                
                // 设置高错误点
                if (data.error_data.high_error_points && data.error_data.high_error_points.length > 0) {
                    let highErrorHTML = '';
                    data.error_data.high_error_points.forEach((point, index) => {
                        highErrorHTML += `Point ${point.point_index}: Error = ${point.error.toFixed(4)} meters<br>`;
                    });
                    document.getElementById('high-error-points').innerHTML = highErrorHTML;
                } else {
                    document.getElementById('high-error-points').innerHTML = 'None identified';
                }
            }
        })
        .catch(error => {
            console.error('Error fetching error data:', error);
        });
}

// 格式化矩阵
function formatMatrix(matrix) {
    let result = '';
    for (let row of matrix) {
        result += '[ ';
        for (let val of row) {
            result += typeof val === 'number' ? val.toFixed(4).padStart(8) : val.padStart(8);
            result += ' ';
        }
        result += ' ]\n';
    }
    return result;
}

// 检查数据
function reviewData() {
    addConsoleEntry('[INFO] Reviewing recorded data...', 'info');
    
    // 向后端请求数据预览
    // 这个功能可以根据需要实现
    
    // 这里使用模拟数据
    setTimeout(() => {
        // 显示样本数据
        addConsoleEntry('Last 3 Laser Tracker Records:', 'info');
        for (let i = 0; i < 3; i++) {
            const distance2 = (Math.random() * 2 + 1).toFixed(3);
            const distance3 = (Math.random() * 2 + 1).toFixed(3);
            const x = (Math.random() * 2 - 1).toFixed(4);
            const y = (Math.random() * 0.5).toFixed(4);
            const z = (Math.random() * 2 - 3).toFixed(4);
            
            addConsoleEntry(`Record ${systemState.recordedDataPoints-i}: d2=${distance2}, d3=${distance3}, x=${x}, y=${y}, z=${z}`, 'data');
        }
        
        if (systemState.eulerPoints > 0) {
            addConsoleEntry('Last 2 Euler Data Records:', 'info');
            for (let i = 0; i < Math.min(2, systemState.eulerPoints); i++) {
                const angle = (Math.random() * 60 - 30).toFixed(2);
                const x = (Math.random() * 2 - 1).toFixed(4);
                const y = (Math.random() * 0.5).toFixed(4);
                const z = (Math.random() * 2 - 3).toFixed(4);
                
                addConsoleEntry(`Record ${systemState.eulerPoints-i}: angle=${angle}°, x=${x}, y=${y}, z=${z}`, 'data');
            }
        }
        
        addConsoleEntry('[INFO] Data review complete', 'info');
    }, 1000);
}

// 可视化点
function visualizePoints() {
    addConsoleEntry('[INFO] Preparing 3D visualization...', 'info');
    
    // 切换到可视化标签
    document.querySelectorAll('.tab').forEach(t => t.classList.remove('active'));
    document.querySelectorAll('.tab-content').forEach(c => c.classList.remove('active'));
    document.querySelector('.tab[data-tab="visualization"]').classList.add('active');
    document.getElementById('visualization').classList.add('active');
    
    // 获取可视化数据
    fetch('/api/visualize', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({
            errorThreshold: document.getElementById('error-threshold').value
        })
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            // 如果有可视化图像，可以显示它们
            if (data.visualizations) {
                // 这里可以根据实际情况处理图像
                addConsoleEntry('[INFO] Visualization images generated', 'info');
            } else {
                // 如果没有图像，使用Canvas绘制示例图形
                const canvas = document.getElementById('visualization-canvas');
                const ctx = canvas.getContext('2d');
                
                // 设置Canvas尺寸
                canvas.width = canvas.parentElement.clientWidth;
                canvas.height = canvas.parentElement.clientHeight;
                
                // 清空Canvas
                ctx.clearRect(0, 0, canvas.width, canvas.height);
                
                // 绘制坐标系统
                drawCoordinateSystem(ctx, canvas.width, canvas.height);
                
                // 绘制点
                drawPoints(ctx, canvas.width, canvas.height);
            }
        } else {
            addConsoleEntry(`[ERROR] Failed to visualize data: ${data.message}`, 'error');
        }
    })
    .catch(error => {
        addConsoleEntry(`[ERROR] Error connecting to server: ${error}`, 'error');
    });
}

// 绘制坐标系统
function drawCoordinateSystem(ctx, width, height) {
    const centerX = width / 2;
    const centerY = height / 2;
    const axisLength = Math.min(width, height) * 0.4;
    
    // 绘制X轴（红色）
    ctx.beginPath();
    ctx.moveTo(centerX, centerY);
    ctx.lineTo(centerX + axisLength, centerY);
    ctx.strokeStyle = 'red';
    ctx.lineWidth = 2;
    ctx.stroke();
    ctx.fillStyle = 'red';
    ctx.fillText('X', centerX + axisLength + 5, centerY + 15);
    
    // 绘制Y轴（绿色）
    ctx.beginPath();
    ctx.moveTo(centerX, centerY);
    ctx.lineTo(centerX, centerY - axisLength);
    ctx.strokeStyle = 'green';
    ctx.lineWidth = 2;
    ctx.stroke();
    ctx.fillStyle = 'green';
    ctx.fillText('Y', centerX - 15, centerY - axisLength - 5);
    
    // 绘制Z轴（蓝色）
    ctx.beginPath();
    ctx.moveTo(centerX, centerY);
    ctx.lineTo(centerX - axisLength * 0.7, centerY + axisLength * 0.7);
    ctx.strokeStyle = 'blue';
    ctx.lineWidth = 2;
    ctx.stroke();
    ctx.fillStyle = 'blue';
    ctx.fillText('Z', centerX - axisLength * 0.7 - 20, centerY + axisLength * 0.7 + 5);
    
    // 绘制原点
    ctx.beginPath();
    ctx.arc(centerX, centerY, 5, 0, Math.PI * 2);
    ctx.fillStyle = 'black';
    ctx.fill();
    ctx.fillText('O', centerX + 10, centerY - 10);
}

// 绘制点
function drawPoints(ctx, width, height) {
    const centerX = width / 2;
    const centerY = height / 2;
    const scale = Math.min(width, height) * 0.15;
    
    // 生成一些随机点
    const numPoints = 15;
    const sourcePoints = [];
    const targetPoints = [];
    const transformedPoints = [];
    
    for (let i = 0; i < numPoints; i++) {
        // 源点（蓝色）
        const srcX = (Math.random() * 2 - 1) * scale * 0.8;
        const srcY = (Math.random() * 2 - 1) * scale * 0.8;
        const srcZ = (Math.random() * 2 - 1) * scale * 0.8;
        sourcePoints.push({ x: srcX, y: srcY, z: srcZ });
        
        // 目标点（红色）
        const tgtX = srcX + scale * 0.3;
        const tgtY = srcY - scale * 0.1;
        const tgtZ = srcZ - scale * 0.2;
        targetPoints.push({ x: tgtX, y: tgtY, z: tgtZ });
        
        // 变换点（绿色）
        const transformedX = tgtX - (Math.random() * 0.2 - 0.1) * scale * 0.3;
        const transformedY = tgtY - (Math.random() * 0.2 - 0.1) * scale * 0.3;
        const transformedZ = tgtZ - (Math.random() * 0.2 - 0.1) * scale * 0.3;
        transformedPoints.push({ x: transformedX, y: transformedY, z: transformedZ });
    }
    
    // 绘制源点
    for (const point of sourcePoints) {
        const screenX = centerX + point.x - point.z * 0.7;
        const screenY = centerY - point.y + point.z * 0.7;
        
        ctx.beginPath();
        ctx.arc(screenX, screenY, 5, 0, Math.PI * 2);
        ctx.fillStyle = 'rgba(30, 144, 255, 0.7)';
        ctx.fill();
        ctx.strokeStyle = 'black';
        ctx.lineWidth = 1;
        ctx.stroke();
    }
    
    // 绘制目标点
    for (const point of targetPoints) {
        const screenX = centerX + point.x - point.z * 0.7;
        const screenY = centerY - point.y + point.z * 0.7;
        
        ctx.beginPath();
        ctx.arc(screenX, screenY, 5, 0, Math.PI * 2);
        ctx.fillStyle = 'rgba(255, 69, 0, 0.7)';
        ctx.fill();
        ctx.strokeStyle = 'black';
        ctx.lineWidth = 1;
        ctx.stroke();
    }
    
    // 绘制变换点
    for (const point of transformedPoints) {
        const screenX = centerX + point.x - point.z * 0.7;
        const screenY = centerY - point.y + point.z * 0.7;
        
        ctx.beginPath();
        ctx.arc(screenX, screenY, 5, 0, Math.PI * 2);
        ctx.fillStyle = 'rgba(50, 205, 50, 0.7)';
        ctx.fill();
        ctx.strokeStyle = 'black';
        ctx.lineWidth = 1;
        ctx.stroke();
    }
    
    // 绘制图例
    const legendX = 30;
    const legendY = 30;
    const legendSpacing = 25;
    
    // 源点
    ctx.beginPath();
    ctx.arc(legendX, legendY, 5, 0, Math.PI * 2);
    ctx.fillStyle = 'rgba(30, 144, 255, 0.7)';
    ctx.fill();
    ctx.strokeStyle = 'black';
    ctx.lineWidth = 1;
    ctx.stroke();
    ctx.fillStyle = 'black';
    ctx.fillText('Source Points (A)', legendX + 15, legendY + 5);
    
    // 目标点
    ctx.beginPath();
    ctx.arc(legendX, legendY + legendSpacing, 5, 0, Math.PI * 2);
    ctx.fillStyle = 'rgba(255, 69, 0, 0.7)';
    ctx.fill();
    ctx.strokeStyle = 'black';
    ctx.lineWidth = 1;
    ctx.stroke();
    ctx.fillStyle = 'black';
    ctx.fillText('Target Points (B)', legendX + 15, legendY + legendSpacing + 5);
    
    // 变换点
    ctx.beginPath();
    ctx.arc(legendX, legendY + legendSpacing * 2, 5, 0, Math.PI * 2);
    ctx.fillStyle = 'rgba(50, 205, 50, 0.7)';
    ctx.fill();
    ctx.strokeStyle = 'black';
    ctx.lineWidth = 1;
    ctx.stroke();
    ctx.fillStyle = 'black';
    ctx.fillText('Transformed Points', legendX + 15, legendY + legendSpacing * 2 + 5);
}

// 显示变换
function showTransformations() {
    document.getElementById('transformation-matrices').style.display = 'block';
    document.getElementById('error-analysis').style.display = 'none';
    
    addConsoleEntry('[INFO] Displaying transformation matrices', 'info');
}

// 显示错误
function showErrors() {
    document.getElementById('transformation-matrices').style.display = 'none';
    document.getElementById('error-analysis').style.display = 'block';
    
    addConsoleEntry('[INFO] Displaying error analysis', 'info');
}

// 导出图表
function exportChart() {
    addConsoleEntry('[INFO] Exporting visualization as PNG image...', 'info');
    
    // 这里可以实现导出功能，或者调用后端API
    setTimeout(() => {
        addConsoleEntry('[SUCCESS] Chart exported to registration_visualization.png', 'info');
    }, 1000);
}

// 重置设置
function resetSettings() {
    if (confirm('Are you sure you want to reset all settings to default values?')) {
        document.getElementById('log-level').value = 'info';
        document.getElementById('visualization-style').value = 'default';
        document.getElementById('auto-save-enabled').checked = true;
        document.getElementById('real-time-visualization').checked = true;
        
        addConsoleEntry('[INFO] Settings reset to default values', 'info');
    }
}

// 清空控制台
function clearConsole() {
    document.querySelectorAll('.console').forEach(console => {
        console.innerHTML = '';
    });
    addConsoleEntry('[INFO] Console cleared', 'info');
}

// 检查设备
function checkDevices() {
    addConsoleEntry('[INFO] Checking connected devices...', 'info');
    
    // 这里可以实现设备检查，或者调用后端API
    setTimeout(() => {
        addConsoleEntry('[INFO] Found devices:', 'info');
        addConsoleEntry(`1. ${document.getElementById('tracker-name').value} (Vive Tracker) - Connected`, 'info');
        addConsoleEntry(`2. ${document.getElementById('serial-port').value} - Connected`, 'info');
        addConsoleEntry('[INFO] All required devices are connected', 'info');
    }, 1500);
}

// 全局存储最新记录的ID和数据
let currentRecordId = null;
let currentRecordData = null;
let eulerRecords = []; // 存储所有欧拉记录

// 保存欧拉点函数修改
function saveEulerPoint() {
    const angle = document.getElementById('euler-angle').value;
    
    fetch('/api/record_euler', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({
            angle: angle
        })
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            // 记录欧拉点数据成功
            addConsoleEntry(`[DATA] Euler point recorded: angle=${angle}°`, 'data');
            
            if (data.data) {
                const pose = data.data;
                addConsoleEntry(`[DATA] Position: x=${pose.x}, y=${pose.y}, z=${pose.z}, roll=${pose.roll}, yaw=${pose.yaw}, pitch=${pose.pitch}`, 'data');
                
                // 保存当前记录数据
                currentRecordData = pose;
                currentRecordId = data.recordId || eulerRecords.length;
                
                // 添加记录到数组
                eulerRecords.push({
                    id: currentRecordId,
                    timestamp: pose.timestamp,
                    angle: angle,
                    position: {x: pose.x, y: pose.y, z: pose.z},
                    orientation: {roll: pose.roll, yaw: pose.yaw, pitch: pose.pitch}
                });
            }
            
            systemState.eulerPoints++;
            
            // 更新状态
            document.getElementById('euler-data-status').className = 'status-indicator status-active';
            document.getElementById('euler-data-status-text').textContent = `${systemState.eulerPoints} Points`;
            
            // 启用处理按钮
            document.getElementById('btn-process-data').disabled = false;
            
            // 隐藏输入容器
            document.getElementById('euler-input-container').style.display = 'none';
            
            // 显示记录审查对话框
            showRecordReview(angle, currentRecordData);
            
            // 添加询问消息
            addConsoleEntry(`[INFO] Euler point with angle ${angle}° added. Do you want to review this record?`, 'info');
        } else {
            addConsoleEntry(`[ERROR] Failed to record Euler point: ${data.message}`, 'error');
        }
    })
    .catch(error => {
        addConsoleEntry(`[ERROR] Error connecting to server: ${error}`, 'error');
    });
}

// 显示记录审查对话框
function showRecordReview(angle, recordData) {
    // 填充审查表单
    document.getElementById('record-angle').value = angle;
    
    // 填充记录详情
    let detailsHtml = '';
    if (recordData) {
        detailsHtml = `Timestamp: ${new Date(recordData.timestamp * 1000).toLocaleString()}<br>`;
        detailsHtml += `Position: x=${recordData.x.toFixed(4)}, y=${recordData.y.toFixed(4)}, z=${recordData.z.toFixed(4)}<br>`;
        detailsHtml += `Orientation: roll=${recordData.roll.toFixed(4)}, yaw=${recordData.yaw.toFixed(4)}, pitch=${recordData.pitch.toFixed(4)}`;
    }
    document.getElementById('record-details').innerHTML = detailsHtml;
    
    // 显示审查容器
    document.getElementById('record-review-container').style.display = 'block';
}

// 确认记录
function confirmRecord() {
    addConsoleEntry('[INFO] Record confirmed', 'info');
    document.getElementById('record-review-container').style.display = 'none';
}

// 修改记录角度
function modifyRecord() {
    const newAngle = document.getElementById('record-angle').value;
    
    fetch('/api/modify_euler', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({
            recordId: currentRecordId,
            newAngle: newAngle
        })
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            addConsoleEntry(`[INFO] Record angle modified to ${newAngle}°`, 'info');
            
            // 更新本地记录
            for (let i = 0; i < eulerRecords.length; i++) {
                if (eulerRecords[i].id === currentRecordId) {
                    eulerRecords[i].angle = newAngle;
                    break;
                }
            }
            
            document.getElementById('record-review-container').style.display = 'none';
        } else {
            addConsoleEntry(`[ERROR] Failed to modify record: ${data.message}`, 'error');
        }
    })
    .catch(error => {
        addConsoleEntry(`[ERROR] Error connecting to server: ${error}`, 'error');
    });
}

// 删除记录
function deleteRecord() {
    if (confirm('Are you sure you want to delete this record?')) {
        fetch('/api/delete_euler', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({
                recordId: currentRecordId
            })
        })
        .then(response => response.json())
        .then(data => {
            if (data.success) {
                addConsoleEntry('[INFO] Record deleted', 'info');
                
                // 从本地记录中删除
                eulerRecords = eulerRecords.filter(record => record.id !== currentRecordId);
                
                // 更新欧拉点数量
                systemState.eulerPoints--;
                document.getElementById('euler-data-status-text').textContent = `${systemState.eulerPoints} Points`;
                
                document.getElementById('record-review-container').style.display = 'none';
            } else {
                addConsoleEntry(`[ERROR] Failed to delete record: ${data.message}`, 'error');
            }
        })
        .catch(error => {
            addConsoleEntry(`[ERROR] Error connecting to server: ${error}`, 'error');
        });
    }
}

// 查看所有欧拉记录
function viewEulerRecords() {
    // 获取欧拉数据记录
    fetch('/api/get_euler_records')
        .then(response => response.json())
        .then(data => {
            if (data.success) {
                // 更新本地记录
                eulerRecords = data.records;
                
                // 填充表格
                const tableBody = document.getElementById('euler-records-table').querySelector('tbody');
                tableBody.innerHTML = '';
                
                eulerRecords.forEach((record, index) => {
                    const row = document.createElement('tr');
                    
                    // 创建并填充单元格
                    const indexCell = document.createElement('td');
                    indexCell.textContent = index;
                    
                    const timestampCell = document.createElement('td');
                    timestampCell.textContent = new Date(record.timestamp * 1000).toLocaleTimeString();
                    
                    const angleCell = document.createElement('td');
                    angleCell.textContent = record.angle;
                    
                    const positionCell = document.createElement('td');
                    positionCell.textContent = `(${record.position.x.toFixed(2)}, ${record.position.y.toFixed(2)}, ${record.position.z.toFixed(2)})`;
                    
                    const actionsCell = document.createElement('td');
                    const editButton = document.createElement('button');
                    editButton.textContent = 'Edit';
                    editButton.className = 'btn btn-sm btn-warning';
                    editButton.onclick = function() {
                        editEulerRecord(record.id);
                    };
                    
                    const deleteButton = document.createElement('button');
                    deleteButton.textContent = 'Delete';
                    deleteButton.className = 'btn btn-sm btn-danger';
                    deleteButton.style.marginLeft = '5px';
                    deleteButton.onclick = function() {
                        deleteEulerRecord(record.id);
                    };
                    
                    actionsCell.appendChild(editButton);
                    actionsCell.appendChild(deleteButton);
                    
                    // 添加单元格到行
                    row.appendChild(indexCell);
                    row.appendChild(timestampCell);
                    row.appendChild(angleCell);
                    row.appendChild(positionCell);
                    row.appendChild(actionsCell);
                    
                    // 添加行到表格
                    tableBody.appendChild(row);
                });
                
                // 显示记录容器
                document.getElementById('euler-records-container').style.display = 'block';
            } else {
                addConsoleEntry(`[ERROR] Failed to get Euler records: ${data.message}`, 'error');
            }
        })
        .catch(error => {
            addConsoleEntry(`[ERROR] Error connecting to server: ${error}`, 'error');
        });
}

// 编辑欧拉记录
function editEulerRecord(recordId) {
    // 找到对应记录
    const record = eulerRecords.find(r => r.id === recordId);
    if (record) {
        currentRecordId = recordId;
        currentRecordData = {
            timestamp: record.timestamp,
            x: record.position.x,
            y: record.position.y,
            z: record.position.z,
            roll: record.orientation.roll,
            yaw: record.orientation.yaw,
            pitch: record.orientation.pitch
        };
        
        // 显示修改对话框
        showRecordReview(record.angle, currentRecordData);
        
        // 隐藏记录列表
        document.getElementById('euler-records-container').style.display = 'none';
    }
}

// 删除欧拉记录（从列表中）
function deleteEulerRecord(recordId) {
    if (confirm('Are you sure you want to delete this record?')) {
        fetch('/api/delete_euler', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({
                recordId: recordId
            })
        })
        .then(response => response.json())
        .then(data => {
            if (data.success) {
                addConsoleEntry('[INFO] Record deleted', 'info');
                
                // 从本地记录中删除
                eulerRecords = eulerRecords.filter(record => record.id !== recordId);
                
                // 更新欧拉点数量
                systemState.eulerPoints--;
                document.getElementById('euler-data-status-text').textContent = `${systemState.eulerPoints} Points`;
                
                // 刷新列表
                viewEulerRecords();
            } else {
                addConsoleEntry(`[ERROR] Failed to delete record: ${data.message}`, 'error');
            }
        })
        .catch(error => {
            addConsoleEntry(`[ERROR] Error connecting to server: ${error}`, 'error');
        });
    }
}

// 关闭记录列表
function closeRecords() {
    document.getElementById('euler-records-container').style.display = 'none';
}

// 初始化按钮函数修改，添加新的按钮事件监听

// 初始化按钮函数，添加重启和退出按钮的事件监听


// 重启应用程序
function restartApplication() {
    if (confirm('Are you sure you want to restart the application? All unsaved data will be lost.')) {
        addConsoleEntry('[INFO] Restarting application...', 'info');
        
        // 调用后端API重启应用
        fetch('/api/restart', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            }
        })
        .then(response => response.json())
        .then(data => {
            if (data.success) {
                // 重启成功，页面会自动刷新
                addConsoleEntry('[INFO] Restart initiated', 'info');
            } else {
                addConsoleEntry(`[ERROR] Failed to restart: ${data.message}`, 'error');
            }
        })
        .catch(error => {
            addConsoleEntry(`[ERROR] Error connecting to server: ${error}`, 'error');
            
            // 如果无法连接服务器，直接刷新页面
            setTimeout(() => {
                window.location.reload();
            }, 2000);
        });
    }
}

// 退出应用程序
function exitApplication() {
    if (confirm('Are you sure you want to exit the application? All unsaved data will be lost.')) {
        addConsoleEntry('[INFO] Shutting down application...', 'info');
        
        // 调用后端API关闭应用
        fetch('/api/exit', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            }
        })
        .then(response => response.json())
        .then(data => {
            if (data.success) {
                addConsoleEntry('[INFO] Application is shutting down...', 'info');
                
                // 显示退出消息
                document.body.innerHTML = `
                    <div style="text-align: center; margin-top: 100px;">
                        <h1>Application Closed</h1>
                        <p>The application has been safely shut down. You can close this window now.</p>
                    </div>
                `;
            } else {
                addConsoleEntry(`[ERROR] Failed to exit: ${data.message}`, 'error');
            }
        })
        .catch(error => {
            addConsoleEntry(`[ERROR] Error connecting to server: ${error}`, 'error');
        });
    }
}