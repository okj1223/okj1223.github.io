// Project detail page interactions:
// - generated TOC
// - responsive content wrappers
// - snippet card / code example behavior
document.addEventListener('DOMContentLoaded', function() {
  const tocNav = document.getElementById('toc-nav');
  const content = document.querySelector('.project-content');
  
  if (!tocNav || !content) return;
  
  // h2, h3, h4 헤딩 찾기
  const headings = content.querySelectorAll('h2, h3, h4');
  
  if (headings.length === 0) {
    document.getElementById('toc-sidebar').style.display = 'none';
    return;
  }
  
  // TOC HTML 생성
  let tocHTML = '<ul class="toc-list">';
  
  headings.forEach((heading, index) => {
    // 고유 ID 생성 (없으면)
    if (!heading.id) {
      // 제목에서 특수문자 제거하고 kebab-case로 변환
      const text = heading.textContent.trim()
        .toLowerCase()
        .replace(/[^a-z0-9\s-]/g, '') // 특수문자 제거
        .replace(/\s+/g, '-')         // 공백을 하이픈으로
        .replace(/-+/g, '-')          // 연속 하이픈 제거
        .replace(/^-|-$/g, '');       // 앞뒤 하이픈 제거
      
      heading.id = text || 'heading-' + index;
    }
    
    // 헤딩 레벨 확인
    const level = parseInt(heading.tagName.charAt(1));
    const text = heading.textContent.trim();
    
    // 이모지 제거 (TOC에서는 깔끔하게)
    const cleanText = text.replace(/[\u{1F600}-\u{1F64F}]|[\u{1F300}-\u{1F5FF}]|[\u{1F680}-\u{1F6FF}]|[\u{1F1E0}-\u{1F1FF}]|[\u{2600}-\u{26FF}]|[\u{2700}-\u{27BF}]/gu, '').trim();
    
    tocHTML += `<li class="toc-item toc-level-${level}">
      <a href="#${heading.id}" class="toc-link" data-target="${heading.id}">${cleanText}</a>
    </li>`;
  });
  
  tocHTML += '</ul>';
  tocNav.innerHTML = tocHTML;
  
  // 스무스 스크롤 이벤트
  const tocLinks = tocNav.querySelectorAll('.toc-link');
  let activeTargetId = '';

  function keepActiveLinkVisible(activeLink) {
    if (!activeLink || window.innerWidth <= 960) return;

    activeLink.scrollIntoView({
      block: 'nearest',
      inline: 'nearest'
    });
  }

  tocLinks.forEach(link => {
    link.addEventListener('click', function(e) {
      e.preventDefault();
      const targetId = this.getAttribute('data-target');
      const target = document.getElementById(targetId);
      
      if (target) {
        const offsetTop = target.getBoundingClientRect().top + window.pageYOffset - 100;
        window.scrollTo({
          top: offsetTop,
          behavior: 'smooth'
        });
      }
    });
  });
  
  // 스크롤 시 현재 섹션 하이라이트
  function updateActiveLink() {
    let current = '';
    const scrollPos = window.pageYOffset + 150;
    
    headings.forEach(heading => {
      const offsetTop = heading.offsetTop;
      if (offsetTop <= scrollPos) {
        current = heading.id;
      }
    });
    
    if (current === activeTargetId) return;
    activeTargetId = current;

    let activeLink = null;

    // 활성 링크 업데이트
    tocLinks.forEach(link => {
      const isActive = link.getAttribute('data-target') === current;
      link.classList.toggle('active', isActive);
      if (isActive) activeLink = link;
    });

    keepActiveLinkVisible(activeLink);
  }
  
  // 스크롤 이벤트 (throttled)
  let ticking = false;
  window.addEventListener('scroll', function() {
    if (!ticking) {
      requestAnimationFrame(function() {
        updateActiveLink();
        ticking = false;
      });
      ticking = true;
    }
  });
  
  // 초기 활성 링크 설정
  updateActiveLink();

  // 표 → 반응형 스크롤 래퍼 자동 적용
  content.querySelectorAll('table').forEach(table => {
    if (!table.closest('.table-wrap')) {
      const wrap = document.createElement('div');
      wrap.className = 'table-wrap';
      table.parentNode.insertBefore(wrap, table);
      wrap.appendChild(table);
    }
  });

  function cleanContextText(text) {
    return text
      .replace(/\s+/g, ' ')
      .replace(/^\d+(?:\.\d+)*\s*/, '')
      .replace(/^#+\s*/, '')
      .replace(/[.:：]\s*$/, '')
      .trim();
  }

  function findNearestCodeContext(card) {
    let prev = card.previousElementSibling;

    while (prev) {
      if (prev.classList && prev.classList.contains('code-toggle')) {
        prev = prev.previousElementSibling;
        continue;
      }

      if (prev.matches && prev.matches('h2, h3, h4, h5')) {
        const headingText = cleanContextText(prev.textContent || '');
        if (headingText) return headingText;
      }

      if (prev.querySelector) {
        const strong = prev.querySelector('strong');
        if (strong) {
          const strongText = cleanContextText(strong.textContent || '');
          if (strongText && strongText.length <= 90) return strongText;
        }
      }

      if (prev.matches && prev.matches('p')) {
        const rawText = (prev.textContent || '').trim();
        if (rawText.endsWith(':')) {
          const paragraphText = cleanContextText(rawText);
          if (paragraphText && paragraphText.length <= 90) return paragraphText;
        }
      }

      prev = prev.previousElementSibling;
    }

    return '';
  }

  function humanizeLanguage(card, fallback) {
    const code = card.querySelector('code[class*="language-"]');
    const className = code
      ? Array.from(code.classList).find(cls => cls.startsWith('language-'))
      : '';
    const lang = className ? className.replace('language-', '') : (fallback || '').toLowerCase();
    const labels = {
      python: 'Python',
      py: 'Python',
      cpp: 'C++',
      c: 'C',
      bash: 'Bash',
      sh: 'Shell',
      shell: 'Shell',
      yaml: 'YAML',
      yml: 'YAML',
      json: 'JSON',
      javascript: 'JavaScript',
      js: 'JavaScript',
      sql: 'SQL',
      markdown: 'Markdown',
      md: 'Markdown',
      html: 'HTML',
      css: 'CSS',
      text: 'Text',
      txt: 'Text',
      plaintext: 'Text'
    };

    return labels[lang] || (lang ? lang.toUpperCase() : 'Code');
  }

  function getSnippetInfo(card) {
    const pre = card.querySelector('pre');
    const code = pre ? pre.querySelector('code') : null;
    if (!pre || !code) return null;

    const className = Array.from(code.classList).find(cls => cls.startsWith('language-')) || '';
    const lang = className ? className.replace('language-', '').toLowerCase() : 'text';
    const rawText = (code.textContent || '').replace(/\s+$/, '');
    const lines = rawText ? rawText.split('\n') : [];
    const nonEmptyLines = lines.filter(line => line.trim());
    const longestLine = nonEmptyLines.reduce((max, line) => Math.max(max, line.length), 0);

    return {
      card,
      pre,
      code,
      lang,
      rawText,
      lines,
      lineCount: nonEmptyLines.length,
      longestLine,
      context: findNearestCodeContext(card)
    };
  }

  function isTextSnippet(info) {
    return ['text', 'txt', 'plaintext'].includes(info.lang);
  }

  function classifyTextSnippet(info) {
    const haystack = `${info.context} ${info.rawText}`.toLowerCase();
    const hasBoxDrawing = /[┌┐└┘│├┤─]/.test(info.rawText);
    const hasReactionArrow = /(?:->|→)/.test(info.rawText);
    const hasChemicalSpecies = /(NH₃|NO₂|NO\b|SO₂|SO₃|N₂|H₂O|O₂|CO₂|H₂|O₃)/.test(info.rawText);

    if (hasBoxDrawing) return 'diagram';
    if (hasReactionArrow && hasChemicalSpecies) return 'reaction';
    if (info.lineCount <= 2 && /[=≈×÷√ΔΣημρτπ]/.test(info.rawText)) return 'formula';
    if (/calculation|analysis|required|verification|torque|load|battery|power|mtbf/.test(haystack)) return 'calculation';
    if (/parameter|specification|characteristic|comparison|benchmark|result|protocol/.test(haystack)) return 'reference';
    return 'note';
  }

  function describeTextSnippet(kind) {
    const labels = {
      diagram: 'System sketch',
      reaction: 'Reaction equation',
      formula: 'Formula',
      calculation: 'Calculation note',
      reference: 'Technical reference',
      note: 'Technical note'
    };

    return labels[kind] || 'Technical note';
  }

  function isGenericDescriptor(text) {
    return /^(?:toggle code:\s*)?(?:example\s+)?(?:python|bash|shell|json|yaml|c\+\+|c|javascript|sql|markdown|text|code)(?:\s+snippet)?$/i.test(text.trim());
  }

  function extractDescriptor(buttonText) {
    const cleaned = buttonText
      .replace(/^Toggle code:\s*/i, '')
      .replace(/^Example\s+/i, '')
      .replace(/\s+snippet$/i, '')
      .trim();

    return cleaned && !isGenericDescriptor(cleaned) ? cleaned : '';
  }

  function isWeakCodeContext(text) {
    const cleaned = (text || '').trim();
    if (!cleaned) return true;

    const wordCount = cleaned.split(/\s+/).length;
    if (wordCount <= 2) return true;

    return /^(total|encoder|decoder|result|results|summary|overview|method|value|values)$/i.test(cleaned);
  }

  function extractCodeSymbol(info) {
    const firstMeaningfulLine = info.lines.find((line) => {
      const trimmed = line.trim();
      return trimmed && !trimmed.startsWith('#') && !trimmed.startsWith('//');
    });

    if (!firstMeaningfulLine) return '';

    const line = firstMeaningfulLine.trim();
    let match = null;

    if (['bash', 'shell', 'sh'].includes(info.lang)) {
      return line.replace(/\s*\\$/, '').split(/\s+/).slice(0, 3).join(' ');
    }

    if (['json', 'yaml', 'yml'].includes(info.lang)) {
      return `${humanizeLanguage(info.card, info.lang)} payload`;
    }

    match = line.match(/^class\s+([A-Za-z_][\w]*)/);
    if (match) return `${match[1]} class`;

    match = line.match(/^def\s+([A-Za-z_][\w]*)\s*\(/);
    if (match) return `${match[1]}()`;

    match = line.match(/^(?:async\s+)?function\s+([A-Za-z_][\w]*)\s*\(/);
    if (match) return `${match[1]}()`;

    match = line.match(/^enum\s+([A-Za-z_][\w]*)/);
    if (match) return `${match[1]} enum`;

    match = line.match(/^(?:void|int|float|double|bool|char|size_t|auto)\s+([A-Za-z_][\w]*)\s*\(/);
    if (match) return `${match[1]}()`;

    return '';
  }

  function resolveCodeDescriptor(info, buttonText) {
    const explicitDescriptor = extractDescriptor(buttonText || '');
    const codeSymbol = extractCodeSymbol(info);

    if (explicitDescriptor && !isWeakCodeContext(explicitDescriptor)) return explicitDescriptor;
    if (info.context && !isWeakCodeContext(info.context)) return info.context;
    if (codeSymbol) return codeSymbol;
    if (explicitDescriptor) return explicitDescriptor;
    if (info.context) return info.context;
    return '';
  }

  function describeCodeSnippet(info) {
    const langLabel = humanizeLanguage(info.card, info.lang);
    const byLanguage = {
      bash: 'command example',
      shell: 'command example',
      sh: 'command example',
      json: 'message example',
      yaml: 'configuration example',
      yml: 'configuration example',
      sql: 'query example'
    };

    const resolvedDescriptor = resolveCodeDescriptor(info);
    if (resolvedDescriptor) return `example: ${resolvedDescriptor}`;
    return byLanguage[info.lang] || `${langLabel} example`;
  }

  function buildToggleLabels(button, info) {
    const resolvedDescriptor = resolveCodeDescriptor(info, button.textContent || '');
    const descriptor = resolvedDescriptor ? `example: ${resolvedDescriptor}` : describeCodeSnippet(info);

    if (/^(example:|.+ example$|.+ command$|.+ builder$|.+ schema$|.+ snippet$)/i.test(descriptor)) {
      return {
        show: `Show ${descriptor}`,
        hide: `Hide ${descriptor}`
      };
    }

    return {
      show: `Show example: ${descriptor}`,
      hide: `Hide example: ${descriptor}`
    };
  }

  function setToggleButtonLabel(button, isOpen) {
    const labelText = isOpen ? button.dataset.hideLabel : button.dataset.showLabel;
    let label = button.querySelector('.code-toggle-label');

    if (!label) {
      button.textContent = '';
      label = document.createElement('span');
      label.className = 'code-toggle-label';
      button.appendChild(label);
    }

    label.textContent = labelText;
    button.setAttribute('aria-label', labelText);
    button.setAttribute('aria-expanded', String(isOpen));
  }

  function createSnippetHeader(title, meta) {
    const header = document.createElement('div');
    header.className = 'snippet-card-header';

    const titleEl = document.createElement('span');
    titleEl.className = 'snippet-card-title';
    titleEl.textContent = title;
    header.appendChild(titleEl);

    if (meta && meta.toLowerCase() !== title.toLowerCase()) {
      const metaEl = document.createElement('span');
      metaEl.className = 'snippet-card-meta';
      metaEl.textContent = meta;
      header.appendChild(metaEl);
    }

    return header;
  }

  function unwrapSnippetCard(card, info, options) {
    const panel = card.querySelector('.code-toggle-panel');
    if (!panel || !info.pre) return;
    const variantClasses = (options.variant || '').split(/\s+/).filter(Boolean);
    const preservedClasses = Array.from(card.classList).filter(
      cls => cls !== 'code-toggle' && cls !== 'is-open'
    );

    const header = createSnippetHeader(options.title, options.meta);
    const body = document.createElement('div');
    body.className = 'snippet-card-panel';
    body.appendChild(info.pre);

    card.className = [...preservedClasses, 'snippet-card', ...variantClasses]
      .concat(options.kind ? [`snippet-kind-${options.kind}`] : [])
      .join(' ');

    card.innerHTML = '';
    card.appendChild(header);
    card.appendChild(body);
  }

  function shouldInlineCodeSnippet(info) {
    return info.lineCount > 0 && info.lineCount <= 12 && info.longestLine <= 110;
  }

  content.querySelectorAll('.code-toggle').forEach(card => {
    const button = card.querySelector('.code-toggle-button');
    const info = getSnippetInfo(card);
    if (!button || !info) return;

    if (isTextSnippet(info)) {
      const kind = classifyTextSnippet(info);
      unwrapSnippetCard(card, info, {
        title: info.context || describeTextSnippet(kind),
        meta: describeTextSnippet(kind),
        variant: 'snippet-card-text',
        kind
      });
      return;
    }

    if (shouldInlineCodeSnippet(info)) {
      const langLabel = humanizeLanguage(card, info.lang);
      const resolvedDescriptor = resolveCodeDescriptor(info);
      unwrapSnippetCard(card, info, {
        title: resolvedDescriptor || `${langLabel} example`,
        meta: `${langLabel} example`,
        variant: 'snippet-card-code snippet-card-inline'
      });
      return;
    }

    const labels = buildToggleLabels(button, info);
    button.dataset.showLabel = labels.show;
    button.dataset.hideLabel = labels.hide;
    setToggleButtonLabel(button, card.classList.contains('is-open'));
  });

  Array.from(content.querySelectorAll('h2')).forEach(heading => {
    const headingText = cleanContextText(heading.textContent || '');
    if (!/appendix/i.test(headingText)) return;
    if (heading.closest('details')) return;

    const details = document.createElement('details');
    details.className = 'content-fold appendix-fold';

    const summary = document.createElement('summary');
    summary.textContent = `Supplementary section: ${headingText}`;
    details.appendChild(summary);

    heading.parentNode.insertBefore(details, heading);

    let current = heading;
    while (current) {
      const next = current.nextElementSibling;
      details.appendChild(current);

      if (!next) break;
      if (next.tagName === 'H2') break;

      current = next;
    }
  });

  content.querySelectorAll('.code-toggle-button').forEach(button => {
    button.addEventListener('click', function() {
      const card = this.closest('.code-toggle');
      if (!card) return;

      const isOpen = card.classList.toggle('is-open');
      setToggleButtonLabel(this, isOpen);
    });
  });
});
