let canvas = document.getElementById("gameCanvas");
let ctx = canvas.getContext("2d");

let dinoImg = new Image();
dinoImg.src = "assets/dino.png";

let cactusImg = new Image();
cactusImg.src = "assets/cactus.png";

let dino = {
  x: 50,
  y: 150,
  width: 40,
  height: 40,
  velocityY: 0,
  gravity: 0.8,
  jumping: false
};

let cactus = {
  x: 800,
  y: 150,
  width: 30,
  height: 40,
  speed: 6
};

let gameRunning = false;

document.getElementById("startBtn").onclick = function () {
  if (!gameRunning) {
    gameRunning = true;
    requestAnimationFrame(gameLoop);
  }
};

document.addEventListener("keydown", function (e) {
  if ((e.code === "Space" || e.code === "ArrowUp") && !dino.jumping) {
    dino.velocityY = -12;
    dino.jumping = true;
  }
});

function gameLoop() {
  if (!gameRunning) return;

  ctx.clearRect(0, 0, canvas.width, canvas.height);

  // 공룡
  dino.velocityY += dino.gravity;
  dino.y += dino.velocityY;
  if (dino.y > 150) {
    dino.y = 150;
    dino.velocityY = 0;
    dino.jumping = false;
  }

  ctx.drawImage(dinoImg, dino.x, dino.y, dino.width, dino.height);

  // 선인장
  cactus.x -= cactus.speed;
  if (cactus.x + cactus.width < 0) {
    cactus.x = 800;
  }

  ctx.drawImage(cactusImg, cactus.x, cactus.y, cactus.width, cactus.height);

  // 충돌 판정
  if (
    dino.x < cactus.x + cactus.width &&
    dino.x + dino.width > cactus.x &&
    dino.y < cactus.y + cactus.height
  ) {
    alert("Game Over!");
    gameRunning = false;
    cactus.x = 800;
    dino.y = 150;
    return;
  }

  requestAnimationFrame(gameLoop);
}
