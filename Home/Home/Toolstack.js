import React from "react";
import { Col, Row } from "react-bootstrap";
import {
  DiWindows,
  DiAndroid,
  DiJavascript1,
  DiReact,
  DiNodejs,
} from "react-icons/di";
import { SiUbuntu } from "react-icons/si"; // ✅ Ubuntu 아이콘 추가

const techList = [
  { icon: <DiWindows />, label: "Windows" },
  { icon: <SiUbuntu />, label: "Ubuntu" },      // ✅ 리눅스 → 우분투로 교체
  { icon: <DiAndroid />, label: "Android" },
  { icon: <DiJavascript1 />, label: "JavaScript" },
  { icon: <DiReact />, label: "React" },
  { icon: <DiNodejs />, label: "Node.js" },
];


function Techstack() {
  const handleMouseEnter = (el) => {
    el.classList.add("flip");
  };

  const handleMouseLeave = (el) => {
    el.classList.remove("flip");
  };

  return (
    <Row style={{ justifyContent: "center", paddingBottom: "10px" }}>
      {techList.map((tech, idx) => (
        <Col
          xs={2}
          md={1}
          key={idx}
          className="tech-icons1"
          onMouseEnter={(e) => handleMouseEnter(e.currentTarget)}
          onMouseLeave={(e) => handleMouseLeave(e.currentTarget)}
        >
          <div className="flip-inner">
            <div className="flip-front card-face">{tech.icon}</div>
            <div className="flip-back card-face">{tech.label}</div>
          </div>
        </Col>

      ))}
    </Row>
  );
}

export default Techstack;
