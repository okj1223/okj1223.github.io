// ✨ Enhanced JavaScript for your blog - Create this as enhanced-blog.js

document.addEventListener('DOMContentLoaded', function() {
    console.log('🚀 Enhanced blog animations loading...');

    // ✨ Intersection Observer for scroll animations
    const observerOptions = {
        threshold: 0.1,
        rootMargin: '0px 0px -50px 0px'
    };

    const observer = new IntersectionObserver((entries) => {
        entries.forEach(entry => {
            if (entry.isIntersecting) {
                entry.target.classList.add('visible');
                
                // Add staggered animation for skill items
                if (entry.target.classList.contains('skills-section')) {
                    const skillItems = entry.target.querySelectorAll('.skill-item');
                    skillItems.forEach((item, index) => {
                        setTimeout(() => {
                            item.style.opacity = '1';
                            item.style.transform = 'translateY(0) scale(1)';
                        }, index * 100);
                    });
                }
            }
        });
    }, observerOptions);

    // Observe all sections
    document.querySelectorAll('section').forEach(section => {
        observer.observe(section);
    });

    // Observe all h2 elements
    document.querySelectorAll('section > h2').forEach(h2 => {
        observer.observe(h2);
    });

    // ✨ Parallax effect for banner
    let ticking = false;
    
    function updateBanner() {
        const scrolled = window.pageYOffset;
        const banner = document.getElementById('banner');
        if (banner) {
            banner.style.transform = `translateY(${scrolled * 0.5}px)`;
        }
        ticking = false;
    }

    window.addEventListener('scroll', () => {
        if (!ticking) {
            requestAnimationFrame(updateBanner);
            ticking = true;
        }
    });

    // ✨ Enhanced skill item interactions
    const skillItems = document.querySelectorAll('.skill-item');
    skillItems.forEach((item, index) => {
        // Initial state for animation
        item.style.opacity = '0';
        item.style.transform = 'translateY(50px) scale(0.8)';
        item.style.transition = 'all 0.6s cubic-bezier(0.4, 0, 0.2, 1)';
        
        // Enhanced hover effects
        item.addEventListener('mouseenter', function() {
            this.style.transform = 'translateY(-15px) scale(1.1) rotateZ(8deg)';
            this.style.zIndex = '10';
        });
        
        item.addEventListener('mouseleave', function() {
            this.style.transform = 'translateY(0) scale(1) rotateZ(0deg)';
            this.style.zIndex = '1';
        });
    });

    // ✨ Enhanced project card interactions
    const projectItems = document.querySelectorAll('.project-item');
    projectItems.forEach(item => {
        item.addEventListener('mouseenter', function() {
            this.style.transform = 'translateY(-15px) scale(1.02)';
        });
        
        item.addEventListener('mouseleave', function() {
            this.style.transform = 'translateY(0) scale(1)';
        });
    });

    // ✨ Certificate 3D hover effects
    const certCards = document.querySelectorAll('.cert-card');
    certCards.forEach(card => {
        card.addEventListener('mousemove', function(e) {
            const rect = this.getBoundingClientRect();
            const x = e.clientX - rect.left;
            const y = e.clientY - rect.top;
            
            const centerX = rect.width / 2;
            const centerY = rect.height / 2;
            
            const rotateX = (y - centerY) / 10;
            const rotateY = (centerX - x) / 10;
            
            this.style.transform = `perspective(1000px) rotateX(${rotateX}deg) rotateY(${rotateY}deg) scale(1.05)`;
        });
        
        card.addEventListener('mouseleave', function() {
            this.style.transform = 'perspective(1000px) rotateX(0) rotateY(0) scale(1)';
        });
    });

    // ✨ Smooth scrolling for anchor links
    document.querySelectorAll('a[href^="#"]').forEach(anchor => {
        anchor.addEventListener('click', function (e) {
            e.preventDefault();
            const target = document.querySelector(this.getAttribute('href'));
            if (target) {
                target.scrollIntoView({
                    behavior: 'smooth',
                    block: 'start'
                });
            }
        });
    });

    // ✨ Logo hover sequence
    const logos = document.querySelectorAll('.logo-img');
    logos.forEach((logo, index) => {
        logo.addEventListener('mouseenter', function() {
            // Create ripple effect on other logos
            logos.forEach((otherLogo, otherIndex) => {
                if (otherIndex !== index) {
                    setTimeout(() => {
                        otherLogo.style.transform = 'translateY(-2px) scale(1.02)';
                        setTimeout(() => {
                            otherLogo.style.transform = 'translateY(0) scale(1)';
                        }, 200);
                    }, Math.abs(otherIndex - index) * 100);
                }
            });
        });
    });

    // ✨ Career items entrance animation
    const careerItems = document.querySelectorAll('.career-list li');
    const careerObserver = new IntersectionObserver((entries) => {
        entries.forEach(entry => {
            if (entry.isIntersecting) {
                const items = entry.target.querySelectorAll('li');
                items.forEach((item, index) => {
                    setTimeout(() => {
                        item.style.opacity = '1';
                        item.style.transform = 'translateX(0)';
                    }, index * 200);
                });
            }
        });
    });

    const careerSection = document.querySelector('.career-list');
    if (careerSection) {
        careerItems.forEach(item => {
            item.style.opacity = '0';
            item.style.transform = 'translateX(-30px)';
            item.style.transition = 'all 0.6s ease';
        });
        careerObserver.observe(careerSection);
    }

    // ✨ Add loading screen fade out
    document.body.style.opacity = '0';
    setTimeout(() => {
        document.body.style.transition = 'opacity 1s ease-in-out';
        document.body.style.opacity = '1';
    }, 100);

    // ✨ Add floating class to elements
    const elementsToFloat = [
        '.logo-img',
        '.skill-item',
        '.project-item'
    ];

    elementsToFloat.forEach(selector => {
        document.querySelectorAll(selector).forEach((element, index) => {
            element.classList.add('floating-element');
            element.style.animationDelay = `${index * 0.5}s`;
        });
    });

    // ✨ Enhanced CV button effect
    const cvButton = document.querySelector('.cv-button');
    if (cvButton) {
        cvButton.addEventListener('mouseenter', function() {
            this.style.boxShadow = '0 0 30px #00d4ff, 0 0 50px #00d4ff';
        });
        
        cvButton.addEventListener('mouseleave', function() {
            this.style.boxShadow = 'none';
        });
    }

    // ✨ Typing effect for banner title (optional)
    const bannerTitle = document.querySelector('.banner-title');
    if (bannerTitle && bannerTitle.textContent) {
        const originalText = bannerTitle.textContent;
        let isTyping = false;
        
        function startTypingEffect() {
            if (isTyping) return;
            isTyping = true;
            
            bannerTitle.textContent = '';
            bannerTitle.style.borderRight = '3px solid #00d4ff';
            
            let i = 0;
            const typeWriter = () => {
                if (i < originalText.length) {
                    bannerTitle.textContent += originalText.charAt(i);
                    i++;
                    setTimeout(typeWriter, 100);
                } else {
                    setTimeout(() => {
                        bannerTitle.style.borderRight = 'none';
                        isTyping = false;
                    }, 1000);
                }
            };
            
            setTimeout(typeWriter, 500);
        }

        // Uncomment the line below if you want the typing effect
        // setTimeout(startTypingEffect, 1000);
    }

    // ✨ Dynamic theme color based on time
    const hour = new Date().getHours();
    const root = document.documentElement;
    
    if (hour >= 6 && hour < 12) {
        // Morning theme
        root.style.setProperty('--neon-color', '#ff6b6b');
    } else if (hour >= 12 && hour < 18) {
        // Afternoon theme
        root.style.setProperty('--neon-color', '#4ecdc4');
    } else if (hour >= 18 && hour < 22) {
        // Evening theme
        root.style.setProperty('--neon-color', '#45b7d1');
    } else {
        // Night theme
        root.style.setProperty('--neon-color', '#00d4ff');
    }

    console.log('✨ Enhanced blog animations loaded successfully!');
});

// ✨ Performance optimization
const performanceStyle = document.createElement('style');
performanceStyle.textContent = `
    .skill-item { transform: translateZ(0); }
    .project-item { transform: translateZ(0); }
    .cert-card { transform: translateZ(0); }
    .logo-img { transform: translateZ(0); }
`;
document.head.appendChild(performanceStyle);