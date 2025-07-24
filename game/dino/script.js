let dino = document.getElementById("dino");
let startBtn = document.getElementById("startBtn");

let isJumping = false;
let isGameRunning = false;

function jump() {
  if (isJumping) return;
  isJumping = true;

  let position = 0;
  let upInterval = setInterval(() => {
    if (position >= 120) {
      clearInterval(upInterval);
      let downInterval = setInterval(() => {
        if (position <= 0) {
          clearInterval(downInterval);
          isJumping = false;
        } else {
          position -= 25;
          dino.style.bottom = position + "px";
        }
      }, 10);
    } else {
      position += 25;
      dino.style.bottom = position + "px";
    }
  }, 10);
}

function createCactus() {
  const cactus = document.createElement("div");
  cactus.classList.add("cactus");
  document.body.appendChild(cactus);

  let cactusPosition = 800;
  cactus.style.left = cactusPosition + "px";

  let moveInterval = setInterval(() => {
    if (!isGameRunning) {
      clearInterval(moveInterval);
      cactus.remove();
      return;
    }

    cactusPosition -= 10;
    cactus.style.left = cactusPosition + "px";

    let dinoBottom = parseInt(window.getComputedStyle(dino).bottom);

    if (cactusPosition < 80 && cactusPosition > 40 && dinoBottom < 50) {
      clearInterval(moveInterval);
      alert("💥 Game Over");
      isGameRunning = false;
      document.querySelectorAll(".cactus").forEach(c => c.remove());
    }

    if (cactusPosition < -60) {
      clearInterval(moveInterval);
      cactus.remove();
    }
  }, 16); // 더 부드럽고 빠르게 이동

  const nextCactusTime = Math.random() * 900 + 1100; // 1.1~2.0초
  setTimeout(() => {
    if (isGameRunning) createCactus();
  }, nextCactusTime);
}

function startGame() {
  if (isGameRunning) return;
  isGameRunning = true;
  createCactus();
}

// 점프 키
document.addEventListener("keydown", function (e) {
  if (e.code === "Space" || e.key === "ArrowUp") {
    if (isGameRunning) jump();
  }
});

// 시작 버튼
startBtn.addEventListener("click", startGame);
