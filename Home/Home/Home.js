// Home.jsx
import React, { useRef } from "react";
import { Container, Row, Col } from "react-bootstrap";
import Particle from "../Particle";
import Home2 from "./Home2";
import Home1 from "./Home1";
import Type from "./Type";
import InteractiveCube from "./InteractiveCube";
import WellRipple3D from "./WellRipple3D";

function Home() {
  const waterRef = useRef(null);

  return (
    <section>
      {/* 히어로 영역: 배경 = WellRipple3D */}
      <Container
        fluid
        className="home-section"
        id="home"
        style={{
          position: "relative",   // ✅ 배경 절대배치 기준
          overflow: "hidden",     // ✅ 배경 넘침 숨김
          minHeight: "70vh",      // 필요하면 조절
        }}
        onPointerDown={(e) => {
          // 컨테이너 어디를 눌러도 그 지점에서 파문
          waterRef.current?.dropAtScreen(e.clientX, e.clientY);
        }}
      >
        {/* 배경: 물효과 (컨테이너 전체 덮기) */}
        <WellRipple3D
          ref={waterRef}
          style={{
            position: "absolute",
            inset: 0,            // top:0 right:0 bottom:0 left:0
            zIndex: 0,
            pointerEvents: "none", // ✅ 클릭은 컨텐츠가 받고, 좌표는 dropAtScreen으로 전달
          }}
          
        />

        {/* 컨텐츠 */}
        {/* <Particle /> */}
        <Container className="home-content" style={{ position: "relative", zIndex: 1 }}>
          <Row>
            <Col md={7} className="home-header">
              <h1 style={{ paddingBottom: 15 }} className="heading">
                Hi There!{" "}
                <span className="wave" role="img" aria-labelledby="wave">
                  👋🏻
                </span>
              </h1>

              <h1 className="heading-name">
                I'M<strong className="main-name"> HYOJAE JUN</strong>
                <br />
                <br />
                <strong className="sub-name">Software Engineer</strong>
              </h1>

              <div style={{ padding: 10, paddingLeft: 45, textAlign: "left" }}>
                <Type />
              </div>
            </Col>

            <Col
              md={5}
              style={{
                paddingBottom: 20,
                display: "flex",
                justifyContent: "center",
                overflow: "visible",
                position: "relative",
                zIndex: 2,           // ✅ 컨텐츠가 배경 위에
                pointerEvents: "auto",
              }}
            >
              <InteractiveCube size={180} />
            </Col>
          </Row>
        </Container>
      </Container>

      {/* 나머지 섹션 */}
      <Home1 />
      <Home2 />
    </section>
  );
}

export default Home;
