const dino = document.getElementById("dino");
const cactus = document.getElementById("cactus");

document.addEventListener("keydown", function(event) {
  jump();
});

function jump() {
  if (dino.classList != "jump") {
    dino.classList.add("jump");

    setTimeout(function () {
      dino.classList.remove("jump");
    }, 300);
  }
}

let isAlive = setInterval(function () {
  let dinoTop = parseInt(window.getComputedStyle(dino).getPropertyValue("top"));
  let cactusLeft = parseInt(window.getComputedStyle(cactus).getPropertyValue("left"));

  if (cactusLeft < 70 && cactusLeft > 0 && dinoTop >= 140) {
    alert("Game Over!");
  }
}, 10);

setInterval(function () {
  let cactusLeft = parseInt(window.getComputedStyle(cactus).getPropertyValue("left"));
  if (cactusLeft <= -20) {
    cactus.style.left = "600px";
  } else {
    cactus.style.left = cactusLeft - 10 + "px";
  }
}, 50);
