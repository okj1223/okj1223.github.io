import React from "react";
import { Container } from "react-bootstrap";
import Toolstack from "./Toolstack";  // 추가 import
import Techstack from "./Techstack"; 

function Home1() {
  return (
    <Container fluid className="home1-about-section" id="about">
      <h1 className="project-heading">
        Professional <strong className="purple">Skillset </strong>
      </h1>

      <Techstack />

      <h1 className="project-heading">
        <strong className="purple">Tools</strong> I use
      </h1>
      
      <Toolstack />
    </Container>
  );
}

export default Home1;
