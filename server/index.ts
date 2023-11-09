import os from "os";
import cors from "cors";
import express from "express";

const port = 5000;
const app = express();
const networkInterfaces = os.networkInterfaces();

let quadrupedCommands: any = {};
let quadrupedStatus: any = {};

app.use(cors());
app.use(express.json({ limit: "2mb" }));
app.use(express.urlencoded({ extended: true }));

app.get("/", (req, res) => {
  res.send("Quadruped API");
});

app.get("/quadruped", (req, res) => {
  quadrupedStatus = req.query;
  console.log("GET /quadruped");
  res.send(quadrupedCommands);
});

app.post("/quadruped", (req, res) => {
  quadrupedCommands = req.body;
  console.log("POST /quadruped");
  res.json(quadrupedStatus);
});

app.get("/quadruped/status", (req, res) => {
  console.log("GET quadruped/status");
  res.json(quadrupedStatus);
});

app.listen(port, () => {
  console.log(`Server: http://localhost:${port}`);
  for (const key in networkInterfaces) {
    const networkInterface = networkInterfaces[key];
    for (const network of networkInterface as any) {
      if (network.family === "IPv4" && !network.internal) {
        console.log(`Network: http://${network.address}:${port}`);
      }
    }
  }
});
