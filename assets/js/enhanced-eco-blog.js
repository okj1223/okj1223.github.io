// 🌿 Enhanced Eco Blog JavaScript (Optional)
// Create this file as: assets/js/enhanced-eco-blog.js

document.addEventListener('DOMContentLoaded', function() {
    console.log('🌱 Eco theme loaded!');

    // ✨ Simple scroll animations
    const observerOptions = {
        threshold: 0.1,
        rootMargin: '0px 0px -50px 0px'
    };

    const observer = new IntersectionObserver((entries) => {
        entries.forEach(entry => {
            if (entry.isIntersecting) {
                entry.target.style.opacity = '1';
                entry.target.style.transform = 'translateY(0)';
            }
        });
    }, observerOptions);

    // Add simple fade-in animation to sections
    document.querySelectorAll('section').forEach(section => {
        section.style.opacity = '0';
        section.style.transform = 'translateY(30px)';
        section.style.transition = 'all 0.6s ease';
        observer.observe(section);
    });

    // ✨ Enhanced skill hover effects
    const skillItems = document.querySelectorAll('.skill-item');
    skillItems.forEach(item => {
        item.addEventListener('mouseenter', function() {
            this.style.transform = 'translateY(-8px) scale(1.05)';
        });
        
        item.addEventListener('mouseleave', function() {
            this.style.transform = 'translateY(0) scale(1)';
        });
    });

    // ✨ Logo ripple effect
    const logos = document.querySelectorAll('.logo-img');
    logos.forEach((logo, index) => {
        logo.addEventListener('mouseenter', function() {
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

    // ✨ Smooth scrolling for links
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

    // ✨ Career items staggered animation
    const careerItems = document.querySelectorAll('.career-list li');
    const careerObserver = new IntersectionObserver((entries) => {
        entries.forEach(entry => {
            if (entry.isIntersecting) {
                const items = entry.target.querySelectorAll('li');
                items.forEach((item, index) => {
                    setTimeout(() => {
                        item.style.opacity = '1';
                        item.style.transform = 'translateX(0)';
                    }, index * 150);
                });
            }
        });
    });

    const careerSection = document.querySelector('.career-list');
    if (careerSection) {
        careerItems.forEach(item => {
            item.style.opacity = '0';
            item.style.transform = 'translateX(-20px)';
            item.style.transition = 'all 0.5s ease';
        });
        careerObserver.observe(careerSection);
    }

    // ✨ Add gentle page load animation
    document.body.style.opacity = '0';
    setTimeout(() => {
        document.body.style.transition = 'opacity 0.8s ease-in-out';
        document.body.style.opacity = '1';
    }, 100);

    // ✨ Dynamic time-based theme adjustment
    const hour = new Date().getHours();
    const root = document.documentElement;
    
    if (hour >= 6 && hour < 12) {
        // Morning - brighter green
        root.style.setProperty('--eco-green', '#2ecc71');
    } else if (hour >= 12 && hour < 18) {
        // Afternoon - standard green
        root.style.setProperty('--eco-green', '#27ae60');
    } else if (hour >= 18 && hour < 22) {
        // Evening - warmer green
        root.style.setProperty('--eco-green', '#229954');
    } else {
        // Night - deeper green
        root.style.setProperty('--eco-green', '#1e8449');
    }

    console.log('🌿 Eco animations initialized successfully!');
});