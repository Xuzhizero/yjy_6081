const express = require('express');
const cors = require('cors'); // 引入 CORS 中间件
const app = express();  // 初始化 express 应用实例
const http = require('http');
const socketIo = require('socket.io');
const fs = require('fs'); // 使用 promise 版本的 fs
const path = require("path"); // 如果是 CommonJS

// 创建 HTTP 服务器并与 Express 连接
const server = http.createServer(app);

// 初始化 Socket.IO 并配置 CORS
const io = socketIo(server, {
  cors: {
    origin: '*',  // 允许来自该源的请求（假设前端运行在 8080 端口）
    methods: ['GET', 'POST'],         // 允许使用 GET 和 POST 方法
    allowedHeaders: ['Content-Type'], // 允许的请求头
    credentials: true,                // 如果需要使用认证，可以设置为 true
  }
});

// 给特定文件手动设置 ETag / Cache-Control
app.get("/style.json", (req, res) => {
  res.setHeader("Cache-Control", "public, max-age=31536000, immutable");

  res.sendFile(path.join(__dirname, "style1.json"));
});

// 配置 CORS，允许来自特定源的请求
app.use(cors({
  origin: '*', // 允许来自 http://127.0.0.1 的请求
  methods: ['GET', 'POST'],  // 允许的方法
}));













app.use(express.json());

const dgram = require('dgram');  // Node.js 内置模块，用于处理 UDP
const WebSocket = require('ws');
const redis = require('@redis/client');

// 创建 Redis 客户端
const client = redis.createClient({
  url: 'redis://122.224.243.126:6379', // 如果使用其他地址或端口，请在这里修改122.224.243.126:6379
});
// 捕获 Redis 错误
client.on('error', (err) => {
  console.error('Redis error:', err);
  if (err.code === 'ECONNRESET') {
    // 如果是 ECONNRESET 错误，尝试重新连接
    reconnectRedis();
  }
});

async function reconnectRedis() {
  try {
    await client.connect();
    console.log('Redis reconnected');
  } catch (error) {
    console.error('Error reconnecting to Redis:', error);
    //setTimeout(reconnectRedis, 5000); // 5秒后重试
  }
}




const udpPort = 20003;  // 设置 UDP 监听端口
const wssPort_IMU = 18080;   // 设置 WebSocket 服务端口
const wssPort_Radar = 8181;   // 设置 WebSocket 服务端口
const wssPort_CMD = 8282;   // 设置 WebSocket 服务端口

// 连接到 Redis
client.connect().catch((err) => {
  console.error('Redis connection error:', err);
});

// 获取 State 状态
app.get('/api/get-checkbox-status', async (req, res) => {
  try {
    // 获取 Navi field 中的 State
    const state = await client.hGet('Navi', 'State');
    
    // 如果 Redis 中没有该字段，返回 0
    if (state === null) {
      return res.json({ status: '0' });
    }
    
    // 如果存在，返回实际的 State 值
    res.json({ status: state });
  } catch (error) {
    console.error(error);
    res.status(500).send('服务器错误');
  }
});

app.post('/api/set-checkbox-status', async (req, res) => {
  try {
    const { status } = req.body;
	console.log("设置State"+status);
    const success = await client.hSet('Navi', 'State', status.toString());
	console.log("success"+success);
    if (success >=0) {
      io.emit('checkbox-status-changed', status);
      res.json({ success: true });
    } else {
      res.json({ success: false });
    }
  } catch (error) {
    console.error(error);
    res.status(500).send('服务器错误');
  }
});
app.post('/api/set-redis', async (req, res) => {
  try {
    const { key, field, value } = req.body;
    console.log(`设置 Redis => key: ${key}, field: ${field}, value: ${value}`);
    
    const success = await client.hSet(key, field, value.toString());
    console.log("success: " + success);

    if (success >= 0) {
      // 可选：广播这个变化
      io.emit('redis-updated', { key, field, value });
      res.json({ success: true });
    } else {
      res.json({ success: false });
    }
  } catch (error) {
    console.error("设置 Redis 失败:", error);
    res.status(500).send('服务器错误');
  }
});

// 获取 Sudu 状态
app.get('/api/get-Sudu-status', async (req, res) => {
  try {
    // 获取 Navi field 中的 State
    const state = await client.hGet('Navi', 'TargetSpeed');
    
    // 如果 Redis 中没有该字段，返回 0
    if (state === null) {
      return res.json({ status: '0' });
    }
    
    // 如果存在，返回实际的 State 值
    res.json({ status: state });
  } catch (error) {
    console.error(error);
    res.status(500).send('服务器错误');
  }
});

app.post('/api/set-Sudu-status', async (req, res) => {
  try {
    const { status } = req.body;
	console.log("设置Sudu"+status);
    const success = await client.hSet('Navi', 'TargetSpeed', status.toString());
	console.log("success"+success);
    if (success >=0) {
      io.emit('Sudu-status-changed', status);
      res.json({ success: true });
    } else {
      res.json({ success: false });
    }
  } catch (error) {
    console.error(error);
    res.status(500).send('服务器错误');
  }
});


// 获取 BiPeng 状态
app.get('/api/get-BiPeng-status', async (req, res) => {
  try {
    // 获取 Navi field 中的 State
    const state = await client.hGet('Navi', 'CA_sw');
    
    // 如果 Redis 中没有该字段，返回 0
    if (state === null) {
      return res.json({ status: false });
    }
    
    // 如果存在，返回实际的 State 值
    res.json({ status: state });
  } catch (error) {
    console.error(error);
    res.status(500).send('服务器错误');
  }
});


app.post('/api/set-BiPeng-status', async (req, res) => {
  try {
    const { status } = req.body;
	console.log("设置BiPeng"+status);
    const success = await client.hSet('Navi', 'CA_sw', status.toString());
	//console.log("success"+success);
    if (success >=0) {
      io.emit('BiPeng-status-changed', status);
      res.json({ success: true });
    } else {
      res.json({ success: false });
    }
  } catch (error) {
    console.error(error);
    res.status(500).send('服务器错误');
  }
});

// 获取 BiPeng 状态
app.get('/api/History_Gpath', async (req, res) => {
  console.log('/api/History_Gpath');
  try {
    // res.json([
    //   { value: 'bj', label: '北京' },
    //   { value: 'sh', label: '上海' }
    // ]);

    fs.readFile('./GPATH.txt', 'utf8', (err, data) => {
      
      if (err) return res.status(500).json({ error: "读取文件失败" });
      console.log(data);
      res.json(JSON.parse(data));
    });
  } catch (error) {
    console.error(error);
    res.status(500).send('服务器错误');
  }
});



app.post('/api/add_History_Gpath', (req, res) => {
  console.log('/api/add_History_Gpath');

  const { status } = req.body;
  console.log("add_History_Gpath " + status);

  // 读取文件
  fs.readFile('./GPATH.txt', 'utf8', (err, data) => {
    let history = [];
    if (!err && data.trim()) {
      try {
        history = JSON.parse(data);
      } catch (parseErr) {
        console.error("解析文件失败", parseErr);
        return res.status(500).json({ error: "解析文件失败" });
      }
    }

    // 生成新的 label（A, B, C, D...）
    const nextLabel = String.fromCharCode(65 + history.length);

    // 新增记录
    history.push({ value: status, label: nextLabel });

    // 写入文件
    fs.writeFile('./GPATH.txt', JSON.stringify(history, null, 2), 'utf8', (writeErr) => {
      if (writeErr) {
        console.error("写入文件失败", writeErr);
        return res.status(500).json({ error: "写入文件失败" });
      }

      // 发送成功响应
      res.json({ message: "新增成功", success: true });
    });
  });
});







// 新增历史路径 API
app.post('/api/add_History_Gpath11', async (req, res) => {
  console.log('/api/add_History_Gpath');
  try {
    const { value } = req.body; // 获取前端传来的路径数据
    if (!value) return res.status(400).json({ error: "value 不能为空" });
    console.log("add_History_Gpat"+value);

    // 读取文件
    fs.readFile('./GPATH.txt', 'utf8', (err, data) => {
      if (err) return res.status(500).json({ error: "读取文件失败" });

      let history = [];
      if (data.trim()) {
        try {
          history = JSON.parse(data);
        } catch (parseError) {
          return res.status(500).json({ error: "文件格式错误" });
        }
      }

      // 生成新的 label（A, B, C, D...）
      const nextLabel = String.fromCharCode(65 + history.length); // 65 是 'A' 的 ASCII

      // 新增记录
      history.push({ value, label: nextLabel });

      // 写入文件
      fs.writeFile(path, JSON.stringify(history, null, 2), 'utf8', (writeErr) => {
        if (writeErr) return res.status(500).json({ error: "写入文件失败" });
        res.json({ message: "新增成功", newRecord: { value, label: nextLabel } });
      });
    });

  } catch (error) {
    console.error(error);
    res.status(500).json({ error: "服务器错误" });
  }
});


app.listen(3000, () => {
  console.log('服务器正在监听 3000 端口');
});

// 启动服务器
server.listen(3030, () => {
  console.log('服务器正在监听 3000 端口');
});

// 创建 WebSocket 服务器，监听 8080 端口
const wss_IMU = new WebSocket.Server({ port: wssPort_IMU });

const wss_Radar = new WebSocket.Server({ port: wssPort_Radar });

const wss_CMD = new WebSocket.Server({ port: wssPort_CMD });

// 监听 WebSocket 客户端连接
wss_CMD.on('connection', (ws) => {
    console.log('客户端已连接');

       
	ws.on('message', (message) => {
        try {
            console.log('Received message:', message.toString());
            
            // 将收到的数据存入 Redis
            client.hSet('Navi', 'GPath', message.toString())  // 使用 hSet 来设置字段值
                .then(() => {
                    console.log('Successfully updated Redis with GPath');
                })
                .catch(err => {
                    console.error('Error updating Redis:', err);
                });
        } catch (error) {
            console.error('Error parsing WebSocket message:', error);
        }
    });




    // 监听 WebSocket 连接关闭
    ws.on('close', () => {
        console.log('客户端已断开连接');
    });
	
	ws.on('error', (err) => {
        console.error('wss_CMD WebSocket error:', err);
    });
});




// 存储 WebSocket 客户端
let wsClient = null;
// 创建 UDP 服务器并绑定端口
const udpServer = dgram.createSocket('udp4');
udpServer.bind(udpPort, () => {
  console.log(`UDP server listening on port ${udpPort}`);
});

wss_IMU.on('connection', (ws, req) => {
 
  const clientIP = req.connection.remoteAddress;
  console.log('Client connected from IP:', clientIP);
  console.log('Request Headers:', req.headers);
  console.log('Request Path:', req.url); 
  
  
   ws.on('error', (err) => {
        console.error('wss_IMU WebSocket error:', err);
    });
  
  // 定时扫描 Redis 中所有的键
  setInterval(async () => {
    try {
     const dataArray = []; // 存储键对应的值

	// 直接获取 'Navi' 和 'IMU' 键的值
	const keysToFetch = ['Navi', 'IMU','LOST'];

	for (let key of keysToFetch) {
	  // 获取键的类型
	  const type = await client.type(key);
	  
	  let value;
	  if (type === 'hash') {
		// 如果是哈希类型，获取所有字段和值
		value = await client.hGetAll(key);
	  } else {
		// 否则获取键的值（如果是字符串类型）
		value = await client.get(key);
	  }

	  // 将键和值封装为对象
	  dataArray.push({ key, value });
	}

	// 发送数据到前端
	ws.send(JSON.stringify(dataArray));

      //console.log('Data sent to client:', JSON.stringify(dataArray, null, 2));
    } catch (error) {
      console.error('Error fetching data from Redis:', error);
    }
  }, 1000); // 每秒钟刷新一次
});

// // 存储所有 WebSocket 客户端
// const wsClients = [];

// wss_Radar.on('connection', (ws) => {
//   console.log('Client connected via WebSocket');
//    wsClients.push(ws);

//   // 监听 WebSocket 连接关闭
//   ws.on('close', () => {
//     console.log('WebSocket client disconnected');

// 	const index = wsClients.indexOf(ws);
//     if (index !== -1) {
//       wsClients.splice(index, 1);
//     }
	
//   });
//   // 监听 UDP 错误
// 	ws.on('error', (err) => {
// 		console.error('wss_Radar server error:', err);
// 	});
	  
  
// });




const wsClients = new Set();

wss_Radar.on('connection', (ws, req) => {
  wsClients.add(ws);
  console.log('Client connected:', req.connection.remoteAddress);

  ws.on('close', () => wsClients.delete(ws));
  ws.on('error', (err) => console.error('wss_IMU error:', err));
});

// 定时发送数据
setInterval(async () => {
  try {
    const keys = await client.keys('data:*'); // 获取所有 data:* 键
    const dataArray = [];

    for (let key of keys) {
      const hash = await client.hGetAll(key); // 获取哈希数据
      const match = key.match(/^data:(\d+)$/); // 提取数字
      if (!match) continue;
      const target = parseInt(match[1]);

      dataArray.push({
        target,
        ...hash  // 合并字段
      });
    }

    const msg = JSON.stringify(dataArray);
    console.log('Sending data to clients:', msg);

    // 群发给所有客户端
    for (let ws of wsClients) {
      if (ws.readyState === WebSocket.OPEN) {
        ws.send(msg);
      }
    }
  } catch (error) {
    console.error('Error fetching data from Redis:', error);
  }
}, 1000);





// 监听 UDP 数据
udpServer.on('message', (msg, rinfo) => {
  //console.log(`Received message from ${rinfo.address}:${rinfo.port} - ${msg}`);

  // 假设我们收到的数据是 JSON 格式
  try {
    const data = msg.toString();  // 解析 UDP 消息

    // 通过 WebSocket 向所有连接的客户端发送数据
    wsClients.forEach(client => {
      if (client.readyState === WebSocket.OPEN) {
        client.send(data);
        //console.log('Sent data to WebSocket client:', data);
      }
    });
  } catch (error) {
    console.error('Error parsing UDP message:', error);
  }
});

// 启动 UDP 监听
udpServer.on('listening', () => {
  const address = udpServer.address();
  console.log(`UDP server listening on ${address.address}:${address.port}`);
});

// 错误处理
udpServer.on('error', (err) => {
  console.error('UDP server error:', err);
  udpServer.close();
});