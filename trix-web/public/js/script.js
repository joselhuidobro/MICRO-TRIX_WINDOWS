function sendAdsConversion(label = 'GT-TQLVV8PB', value = 1, currency = 'MXN') {
         gtag('event', 'conversion', { send_to: `AW-11070400416/${label}`, value, currency });
       }

       // Carrusel de imágenes
       document.addEventListener('DOMContentLoaded', () => {
         const imgs = [
           { src: '/img/logo_trix_st.webp', alt: 'Logo TRIX Studio' },
           { src: '/img/gabinete.webp', alt: 'Motor Merik 711' },
           { src: '/img/bft200h.webp', alt: 'Motor BFT 200H' }
         ];
         let i = 0;
         const imgTag = document.getElementById('carouselImage');
         if (imgTag) {
           setInterval(() => {
             i = (i + 1) % imgs.length;
             imgTag.src = imgs[i].src;
             imgTag.alt = imgs[i].alt;
           }, 4000);
         }
       });

       // Cotizador flotante
       const quoteIds = ['quoteContainer', 'quoteContainer2', 'quoteContainer3'];

       function initQuote(containerId) {
         const quote = document.getElementById(containerId);
         if (!quote) return;

         const header = quote.querySelector('.quote-header');
         const close = quote.querySelector('.close-quote');
         const body = quote.querySelector('form');

         function toggle(forceClose = false) {
           const minimized = quote.classList.contains('minimized');

           if (minimized && !forceClose) {
             quoteIds.forEach(id => {
               if (id !== containerId) {
                 const otherQuote = document.getElementById(id);
                 if (otherQuote && !otherQuote.classList.contains('minimized')) {
                   otherQuote.classList.add('minimized');
                   otherQuote.querySelector('form').classList.add('hidden');
                 }
               }
             });
           }

           if (!minimized || forceClose) {
             quote.classList.add('minimized');
             body.classList.add('hidden');
           } else {
             quote.classList.remove('minimized');
             body.classList.remove('hidden');
           }
         }

         if (!quote.dataset.bound) {
           header.addEventListener('click', () => toggle());
           close.addEventListener('click', e => { e.stopPropagation(); toggle(true); });
           quote.dataset.bound = 'true';
         }
       }

       function initQuoteMenu() {
         const menuContainer = document.getElementById('quoteMenuContainer');
         if (!menuContainer) {
           console.error('quoteMenuContainer no encontrado');
           return;
         }
         const menuHeader = menuContainer.querySelector('.quote-header');
         const menuList = document.getElementById('quoteMenu');

         menuHeader.addEventListener('click', () => {
           menuContainer.classList.toggle('minimized');
           menuList.classList.toggle('hidden');
         });

         menuList.querySelectorAll('li').forEach(li => {
           li.addEventListener('click', () => {
             const targetId = li.dataset.target;
             const targetQuote = document.getElementById(targetId);
             
             targetQuote.classList.remove('hidden');
             targetQuote.classList.remove('minimized');
             targetQuote.querySelector('form').classList.remove('hidden');
             
             menuContainer.classList.add('minimized');
             menuList.classList.add('hidden');
             
             quoteIds.forEach(id => {
               if (id !== targetId) {
                 const other = document.getElementById(id);
                 other.classList.add('hidden');
                 other.classList.add('minimized');
                 other.querySelector('form').classList.add('hidden');
               }
             });
           });
         });
       }

       document.addEventListener('DOMContentLoaded', () => {
         initQuote('quoteContainer');
         initQuote('quoteContainer2');
         initQuote('quoteContainer3');
         initQuoteMenu();
       });

       // Helper WhatsApp + CTA
       function openWhatsApp(msg, ga4EventName) {
         window.open('https://wa.me/525554539552?text=' + encodeURIComponent(msg), '_blank');
         gtag('event', ga4EventName, { event_category: 'CTA', event_label: msg.split('\n')[0] });
         sendAdsConversion();
       }

       function enviarMensaje() {
         const l = document.getElementById('largo_hoja').value;
         const a = document.getElementById('alto_hoja').value;
         const f = document.getElementById('frecuencia').value;
         const t = document.getElementById('tipo_puerta').value;
         const m = document.getElementById('motor').value;
         const msg = `¡Hola! Estoy interesado en una puerta:\nLargo: ${l} m\nAlto: ${a} m\nFrecuencia: ${f}/h\nTipo: ${t}\nMotor: ${m}`;
         openWhatsApp(msg, 'whatsapp_puerta');
       }

       function enviarMensaje2() {
         const area = document.getElementById('area_render').value;
         const imgs = document.getElementById('num_imagenes').value;
         const entrega = document.getElementById('entrega').value;
         const msg = `¡Hola! Me interesa un render 4K:\nÁrea: ${area} m²\nImágenes: ${imgs}\nEntrega: ${entrega}`;
         openWhatsApp(msg, 'whatsapp_render');
       }

       function enviarMensaje3() {
         const tipo = document.getElementById('tipo_controlador').value;
         const protocolo = document.getElementById('protocolo').value;
         const io = document.getElementById('io_puntos').value;
         const gab = document.getElementById('gabinete').value;
         const msg = `¡Hola! Necesito asesoría para:\n• Controlador: ${tipo}\n• Protocolo: ${protocolo}\n• I/O: ${io}\n• Gabinete: ${gab}`;
         openWhatsApp(msg, 'whatsapp_controladores');
       }

       document.getElementById('callNowBtn')?.addEventListener('click', () => {
         gtag('event', 'call_click', { event_category: 'CTA', event_label: 'Llamar ahora' });
         sendAdsConversion();
       });