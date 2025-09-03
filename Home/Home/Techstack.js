import React from "react";
import { Col, Row } from "react-bootstrap";
import { CgCPlusPlus } from "react-icons/cg";
import { DiPython, DiGit, DiJava } from "react-icons/di";
import { SiTensorflow, SiPytorch, SiDocker } from "react-icons/si";

// SkillIcon 컴포넌트 (2번 방법 적용)
const SkillIcon = ({ iconName, size = 40 }) => {
  return (
    <img
      src={`https://skillicons.dev/icons?i=${iconName}&theme=light`}
      alt={iconName}
      style={{ width: `${size}px`, height: `${size}px` }}
    />
  );
};

const techList = [
  { icon: <DiPython size={40} />, label: "Python" },
  { icon: <SiTensorflow size={40} />, label: "TensorFlow" },
  { icon: <SiPytorch size={40} />, label: "PyTorch" },
  { icon: <DiJava size={40} />, label: "Java" },
  { icon: <CgCPlusPlus size={40} />, label: "C/C++" },
  { icon: <DiGit size={40} />, label: "Git" },
  { icon: <SiDocker size={40} />, label: "Docker" },
  { icon: <SkillIcon iconName="ros" />, label: "ROS2" }, // skillicons.dev의 ROS 아이콘 사용
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
