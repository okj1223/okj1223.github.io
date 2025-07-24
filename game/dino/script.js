let dino = document.getElementById("dino");
let cactus = document.getElementById("cactus");
let startBtn = document.getElementById("startBtn");

let isJumping = false;
let isGameRunning = false;
let cactusInterval;
let gravityInterval;

function jump() {
  if (isJumping) return;
  isJumping = true;

  let position = 0;
  let upInterval = setInterval(() => {
    if (position >= 100) {
      clearInterval(upInterval);
      let downInterval = setInterval(() => {
        if (position <= 0) {
          clearInterval(downInterval);
          isJumping = false;
        } else {
          position -= 20;
          dino.style.bottom = position + "px";
        }
      }, 10);
    } else {
      position += 20;
      dino.style.bottom = position + "px";
    }
  }, 10);
}

function startGame() {
  if (isGameRunning) return;

  isGameRunning = true;
  cactus.style.left = "800px";

  cactusInterval = setInterval(() => {
    let cactusLeft = parseInt(window.getComputedStyle(cactus).left);
    let dinoBottom = parseInt(window.getComputedStyle(dino).bottom);

    if (cactusLeft < 80 && cactusLeft > 40 && dinoBottom < 50) {
      alert("💥 Game Over");
      clearInterval(cactusInterval);
      isGameRunning = false;
      cactus.style.left = "800px";
      return;
    }

    cactus.style.left = cactusLeft - 10 + "px";

    if (cactusLeft <= -20) {
      cactus.style.left = "800px";
    }
  }, 30);
}

// 키 입력
document.addEventListener("keydown", function (e) {
  if (e.code === "Space" || e.key === "ArrowUp") {
    if (isGameRunning) jump();
  }
});

// 버튼 클릭
startBtn.addEventListener("click", startGame);
