let canvas = document.getElementById("gameCanvas");
let ctx = canvas.getContext("2d");
let isRunning = false;

let dino = {
  x: 50,
  y: 150,
  width: 40,
  height: 40,
  dy: 0,
  gravity: 1.5,
  jumpPower: -15,
  grounded: true,
};

function drawDino() {
  ctx.fillStyle = "#222";
  ctx.fillRect(dino.x, dino.y, dino.width, dino.height);
}

function update() {
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  // 중력 적용
  dino.dy += dino.gravity;
  dino.y += dino.dy;

  // 바닥 도달 시 멈춤
  if (dino.y + dino.height >= canvas.height) {
    dino.y = canvas.height - dino.height;
    dino.dy = 0;
    dino.grounded = true;
  }

  drawDino();

  if (isRunning) {
    requestAnimationFrame(update);
  }
}

// 점프 기능
document.addEventListener("keydown", function (e) {
  if (e.code === "Space" || e.key === "ArrowUp") {
    if (dino.grounded) {
      dino.dy = dino.jumpPower;
      dino.grounded = false;
    }
  }
});

// Start 버튼
document.getElementById("startBtn").addEventListener("click", function () {
  isRunning = true;
  update();
});
