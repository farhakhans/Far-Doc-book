// Utility function to add animations to headings
export const animateHeadings = () => {
  // Function to animate headings when they come into view
  const observerOptions = {
    threshold: 0.1,
    rootMargin: '0px 0px -50px 0px'
  };

  const headingObserver = new IntersectionObserver((entries) => {
    entries.forEach(entry => {
      if (entry.isIntersecting) {
        const element = entry.target;
        element.style.opacity = '0';
        element.style.transform = 'translateY(20px)';

        // Small delay to ensure the style is applied before animation
        setTimeout(() => {
          element.style.transition = 'opacity 0.6s ease-out, transform 0.6s ease-out';
          element.style.opacity = '1';
          element.style.transform = 'translateY(0)';
        }, 10);
      }
    });
  }, observerOptions);

  // Observe all headings
  const headings = document.querySelectorAll('h1, h2, h3, h4, h5, h6');
  headings.forEach(heading => {
    // Only observe if not already animated
    if (!heading.classList.contains('animated')) {
      heading.classList.add('animated');
      heading.style.opacity = '0';
      headingObserver.observe(heading);
    }
  });

  // Also animate paragraphs and lists
  const observerOptionsForContent = {
    threshold: 0.1,
    rootMargin: '0px 0px -50px 0px'
  };

  const contentObserver = new IntersectionObserver((entries) => {
    entries.forEach(entry => {
      if (entry.isIntersecting) {
        const element = entry.target;
        if (!element.classList.contains('animated-content')) {
          element.classList.add('animated-content');
          element.style.opacity = '0';
          element.style.transform = 'translateY(10px)';

          setTimeout(() => {
            element.style.transition = 'opacity 0.5s ease-out, transform 0.5s ease-out';
            element.style.opacity = '1';
            element.style.transform = 'translateY(0)';
          }, 10);
        }
      }
    });
  }, observerOptionsForContent);

  // Observe paragraphs and lists
  const paragraphs = document.querySelectorAll('p');
  const lists = document.querySelectorAll('ul, ol');
  [...paragraphs, ...lists].forEach(element => {
    if (!element.classList.contains('animated-content')) {
      contentObserver.observe(element);
    }
  });
};

// Function to animate navigation items
export const animateNavigation = () => {
  const navbarItems = document.querySelectorAll('.navbar__item, .navbar__link, .navbar__brand');
  navbarItems.forEach((item, index) => {
    if (!item.classList.contains('animated-nav')) {
      item.classList.add('animated-nav');
      item.style.opacity = '0';
      item.style.transform = 'translateX(-20px)';

      setTimeout(() => {
        item.style.transition = 'opacity 0.5s ease-out, transform 0.5s ease-out';
        item.style.opacity = '1';
        item.style.transform = 'translateX(0)';
      }, 100 + (index * 50)); // Stagger the animations
    }
  });
};

// Initialize animations when DOM is loaded
export const initAnimations = () => {
  if (typeof window !== 'undefined' && typeof document !== 'undefined') {
    // Run animations when page loads
    if (document.readyState === 'loading') {
      document.addEventListener('DOMContentLoaded', () => {
        animateHeadings();
        animateNavigation();
      });
    } else {
      // If DOM is already loaded, run immediately
      animateHeadings();
      animateNavigation();
    }

    // Also run when React components update
    setTimeout(() => {
      animateHeadings();
      animateNavigation();
    }, 100);
  }
};