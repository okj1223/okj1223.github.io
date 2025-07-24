let dino = document.getElementById("dino");
let startBtn = document.getElementById("startBtn");

let isJumping = false;
let isGameRunning = false;
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

    if (cactusPosition < -60) {
      clearInterval(moveInterval);
      cactus.remove();
    } else {
      cactusPosition -= 10;
      cactus.style.left = cactusPosition + "px";

      let dinoBottom = parseInt(window.getComputedStyle(dino).bottom);

      if (cactusPosition < 80 && cactusPosition > 40 && dinoBottom < 50) {
        clearInterval(moveInterval);
        alert("💥 Game Over");
        isGameRunning = false;
        document.querySelectorAll(".cactus").forEach(c => c.remove());
      }
    }
  }, 20);

  const nextCactusTime = Math.random() * 1000 + 1200; // 1.2~2.2초
  setTimeout(createCactus, nextCactusTime);
}

function startGame() {
  if (isGameRunning) return;
  isGameRunning = true;
  createCactus(); // 🔥 여기 추가!
}



// 키 입력
document.addEventListener("keydown", function (e) {
  if (e.code === "Space" || e.key === "ArrowUp") {
    if (isGameRunning) jump();
  }
});

// 버튼 클릭
startBtn.addEventListener("click", startGame);
