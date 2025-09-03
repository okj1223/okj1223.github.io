import React, { useEffect, useState } from "react";

const strings = [
  "· AI / Machine Learning",
  "· Computer Vision",
  "· Autonomous Robotics",
  "· Smart Factory",
];

export default function Type() {
  const [displayText, setDisplayText] = useState(""); // 타이핑 누적 텍스트
  const [stringIndex, setStringIndex] = useState(0); // 타이핑 중 문장 인덱스
  const [charIndex, setCharIndex] = useState(0); // 문장 내 글자 인덱스
  const [phase, setPhase] = useState("typing"); // typing, whiteErase, purpleFill
  const [eraseFillIndex, setEraseFillIndex] = useState(0); // 덮어쓰기 진행 인덱스

  const fullText = strings.join("\n");

  useEffect(() => {
    let timeout;

    if (phase === "typing") {
      // 처음 타이핑은 한 번만
      if (stringIndex >= strings.length) {
        // 타이핑 끝났으면 다음 단계로
        timeout = setTimeout(() => {
          setPhase("whiteErase");
          setEraseFillIndex(0);
        }, 800);
      } else {
        if (charIndex <= strings[stringIndex].length) {
          timeout = setTimeout(() => {
            setDisplayText((prev) =>
              stringIndex === 0
                ? strings[stringIndex].slice(0, charIndex)
                : prev + strings[stringIndex].charAt(charIndex - 1)
            );
            setCharIndex((prev) => prev + 1);
          }, 100);
        } else {
          setDisplayText((prev) => prev + "\n");
          setCharIndex(0);
          setStringIndex((prev) => prev + 1);
        }
      }
    } else if (phase === "whiteErase") {
      if (eraseFillIndex <= fullText.length) {
        timeout = setTimeout(() => {
          setEraseFillIndex((prev) => prev + 1);
        }, 50);
      } else {
        timeout = setTimeout(() => {
          setPhase("purpleFill");
          setEraseFillIndex(0);
        }, 400);
      }
    } else if (phase === "purpleFill") {
      if (eraseFillIndex <= fullText.length) {
        timeout = setTimeout(() => {
          setEraseFillIndex((prev) => prev + 1);
        }, 50);
      } else {
        timeout = setTimeout(() => {
          // 타이핑 다시 안 하고, 덮어쓰기만 반복하므로 displayText 유지
          setPhase("whiteErase");
          setEraseFillIndex(0);
        }, 400);
      }
    }

    return () => clearTimeout(timeout);
  }, [phase, stringIndex, charIndex, eraseFillIndex, fullText.length]);

  const renderText = () => {
    if (phase === "typing") {
      return (
        <pre
          style={{ margin: 0, color: "#be6adf", whiteSpace: "pre-wrap" }}
        >
          {displayText}
        </pre>
      );
    } else if (phase === "whiteErase") {
      return (
        <pre
          style={{
            margin: 0,
            color: "#be6adf",
            whiteSpace: "pre-wrap",
            position: "relative",
          }}
        >
          {fullText.split("").map((char, i) => (
            <span key={i} style={{ color: i < eraseFillIndex ? "white" : "#be6adf" }}>
              {char}
            </span>
          ))}
        </pre>
      );
    } else if (phase === "purpleFill") {
      return (
        <pre
          style={{
            margin: 0,
            color: "white",
            whiteSpace: "pre-wrap",
            position: "relative",
          }}
        >
          {fullText.split("").map((char, i) => (
            <span key={i} style={{ color: i < eraseFillIndex ? "#be6adf" : "white" }}>
              {char}
            </span>
          ))}
        </pre>
      );
    }
  };

  return (
    <div
      style={{
        fontSize: "2.2em",
        fontWeight: 600,
        fontFamily: "'Courier New', Courier, monospace",
        lineHeight: 1.3,

        minHeight: "4.7em",  // 4줄 × 1.3(line-height) × 약 1.25em(글자 크기 비례) -> 대략 이 정도
        // 또는 height: "6.5em" 으로 고정해도 됨
        overflow: "hidden", // 넘쳐도 안 밀리도록
      }}
    >
      {renderText()}
    </div>
  );
}
