#ifndef WEBSERVERHELPER_H
#define WEBSERVERHELPER_H

#if MICRO_VERSION == 1
 #include <WiFi.h>
 #include <ESPAsyncWebServer.h>
#endif
#if MICRO_VERSION == 2
 #include <AsyncWebServer_Teensy41.h>
 File tempFile;
#endif
void stop(){aog.stop();}

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
bool isWebServerOn = true;

// Server ############################################################################################################
// Make size of files human readable
// source: https://github.com/CelliesProjects/minimalUploadAuthESP32
String humanReadableSize(const size_t bytes) {
  if (bytes < 1024) return String(bytes) + " B";
  else if (bytes < (1024 * 1024)) return String(bytes / 1024.0) + " KB";
  else if (bytes < (1024 * 1024 * 1024)) return String(bytes / 1024.0 / 1024.0) + " MB";
  else return String(bytes / 1024.0 / 1024.0 / 1024.0) + " GB";
}

// used by server.on functions to discern whether a user has the correct httpapitoken OR is authenticated by username and password
bool checkUserWebAuth(AsyncWebServerRequest * request) {
  bool isAuthenticated = false;

  if (request->authenticate("admin", "admin")) {
    Serial.println("is authenticated via username and password");
    isAuthenticated = true;
  }
  return isAuthenticated;
}

// handles uploads to the filserver
void handleUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
  if (!checkUserWebAuth(request)) return request->requestAuthentication();// make sure authenticated before allowing upload
  String fn = "/"+filename;
 #if MICRO_VERSION == 1
  if (!index) request->_tempFile = db.open(fn.c_str(), 1);
  if (len) request->_tempFile.write(data, len);
  if (final) request->_tempFile.close();
 #endif
 #if MICRO_VERSION == 2
  if (!index) tempFile = db.open(fn.c_str(), 1);
  if (len) tempFile.write(data, len);
  if (final) tempFile.close();
 #endif
}

void notFound(AsyncWebServerRequest *request) {
  String logmessage = "Client:" + String(request->client()->remoteIP()) + " " + request->url();
  Serial.println(logmessage);
  request->send(404, "text/plain", "Not found");
}

// list all of the files, if ishtml=true, return html rather than simple json
String listFiles(bool ishtml = true) {
  String returnText = "";
  //Serial.println("Listing files stored on Filesystem");
  File root = db.fs->open("/");
  File foundfile = root.openNextFile();
  if (ishtml) {
    returnText += "<table><tr><th align='left'>Name</th><th align='left'>Size</th><th></th><th></th></tr>";
  }else returnText += "{\"data\":{\"files\":[";
  while (foundfile) {
    if (ishtml) {
      returnText += "<tr align='left'><td>" + String(foundfile.name()) + "</td><td>" + humanReadableSize(foundfile.size()) + "</td>";
      returnText += "<td><button onclick=\"downloadDeleteButton(\'" + String(foundfile.name()) + "\', \'download\')\">Download</button>";
      returnText += "<td><button onclick=\"downloadDeleteButton(\'" + String(foundfile.name()) + "\', \'delete\')\">Delete</button></tr>";
    } else {
      returnText += "{\"name\":\"" + String(foundfile.name()) + "\",\"size\":\"" + humanReadableSize(foundfile.size()) + "\"},";
    }
    foundfile = root.openNextFile();
  }
  if (ishtml) {
    returnText += "</table>";
  } else {
    uint16_t last= returnText.length()-1;
    if(last>25) returnText.remove(last);
    returnText += "]}}";
  }
  root.close();
  foundfile.close();
  return returnText;
}

// Webpages ############################################################################################################
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="author" content="Miguel Cebrian">
  <title>AIO Board Configuration</title>
  <link rel="icon" href='data:image/svg+xml;utf8,<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 10.12 13.49"><path d="M5.06 13.5s5.06-4.8 5.06-8.43C10.12 2.27 7.85 0 5.06 0S0 2.27 0 5.06C0 8.7 5.06 13.5 5.06 13.5zm0-5.9c-1.4 0-2.53-1.13-2.53-2.53s1.13-2.53 2.53-2.53 2.53 1.13 2.53 2.53-1.13 2.53-2.53 2.53z" fill="green"/><path d="M3.55 6.51c-.8-.86-.75-2.2.11-2.99s2.2-.75 2.99.11.75 2.2-.11 2.99-2.2.75-2.99-.11zm.21-2.69c-.09.16-.11.4-.03.7.02.08.05.16.08.24.17-.1.33-.22.49-.36zm-.19 1.06c-.04-.1-.07-.2-.1-.3s-.04-.19-.05-.29c-.1.22-.16.46-.17.7a2.2 2.2 0 0 0 .32-.12zm.57.9c-.19-.21-.34-.44-.46-.66-.14.06-.28.11-.42.15.04.35.18.68.4.96l.48-.45zM3.92 5c.12.22.25.42.42.6l.58-.54-.43-.47c-.18.16-.37.29-.56.4zm.76-.58l.43.47.58-.54c-.17-.17-.36-.33-.57-.46a3.6 3.6 0 0 1-.44.53zm-.16 1.38c.18.19.38.34.57.46a3.6 3.6 0 0 1 .44-.53l-.43-.47zm.77-.72l.43.47c.18-.16.37-.29.56-.4-.11-.2-.25-.41-.42-.6zm.03 1.31c.08.04.16.07.23.1.29.1.53.1.7.02l-.54-.58c-.15.15-.28.31-.39.47zm.43.42c-.1-.02-.19-.04-.28-.07-.1-.03-.19-.08-.29-.12-.06.1-.1.2-.14.31.24.01.49-.03.72-.11zm-.8-.33c-.23-.14-.44-.31-.63-.5l-.48.45c.26.24.58.4.93.47a2.4 2.4 0 0 1 .18-.41zm1.84-.64c.1-.22.16-.46.17-.7-.11.03-.22.07-.32.12.04.1.07.2.1.3.03.09.04.19.05.29zm-.88-.11l.54.58c.09-.16.11-.4.03-.7-.02-.08-.05-.16-.08-.24-.18.1-.34.22-.49.36zm.62-.72a2.4 2.4 0 0 1 .42-.15c-.04-.35-.18-.68-.4-.96l-.48.45c.18.2.33.42.46.66zm-.15-1.3c-.26-.24-.58-.4-.93-.47a2.4 2.4 0 0 1-.18.41c.21.13.43.3.63.5zm-1.63-.31a2.4 2.4 0 0 1 .29.12c.06-.1.1-.2.14-.31-.24-.01-.49.03-.71.11.09.02.19.04.28.07zm.15.35c-.08-.04-.15-.07-.23-.1-.29-.1-.53-.1-.7-.02l.54.58c.15-.15.28-.31.39-.47z" fill="navy"/></svg>'>
  <style>
    .facts {
      background: url('https://images.pexels.com/photos/5558299/pexels-photo-5558299.jpeg?auto=compress&cs=tinysrgb&w=1260&h=750&dpr=1');
      padding: 1rem 4rem;
      background-position: center center;
      background-size: cover;
      position: relative;
      background-attachment: fixed;
    }
    .fixed-top,.modal{position:fixed;top:0;left:0}.modal,input[type=file]{display:none}.btn,.form-control[type=file]:not(:disabled):not([readonly]),[role=button],[type=button]:not(:disabled),[type=reset]:not(:disabled),[type=submit]:not(:disabled),button:not(:disabled){cursor:pointer}.modal-header h2,body,input{margin:0}button,input,select{text-transform:none}body{font-family:"Open Sans",sans-serif;color:#272829}:root,[data-bs-theme=light]{--bs-primary:#0d6efd;--bs-secondary:#6c757d;--bs-success:#198754;--bs-info:#0dcaf0;--bs-warning:#ffc107;--bs-danger:#dc3545;--bs-light:#f8f9fa;--bs-border-width:1px;--bs-border-color:#dee2e6;--bs-border-color-translucent:rgba(0, 0, 0, 0.175);--bs-primary-border-subtle:#9ec5fe;--bs-focus-ring-color:rgba(13, 110, 253, 0.25);--bs-border-radius:0.375rem;--bs-border-style:solid;--bs-body-color:#212529;--bs-body-color-rgb:33,37,41;--bs-body-bg:#fff;--bs-border-radius-lg:0.5rem;--bs-box-shadow-sm:0 0.125rem 0.25rem rgba(0, 0, 0, 0.075);--bs-box-shadow:0 0.5rem 1rem rgba(0, 0, 0, 0.15);--bs-secondary-color:rgba(33, 37, 41, 0.75)}.py-0{padding-top:0!important;padding-bottom:0!important}.fixed-top{right:0;z-index:1030}.modal{--bs-modal-zindex:1055;--bs-modal-width:500px;--bs-modal-padding:1rem;--bs-modal-margin:0.5rem;--bs-modal-color: ;--bs-modal-bg:var(--bs-body-bg);--bs-modal-border-color:var(--bs-border-color-translucent);--bs-modal-border-width:var(--bs-border-width);--bs-modal-border-radius:var(--bs-border-radius-lg);--bs-modal-box-shadow:var(--bs-box-shadow-sm);--bs-modal-inner-border-radius:calc(var(--bs-border-radius-lg) - (var(--bs-border-width)));--bs-modal-header-padding-x:1rem;--bs-modal-header-padding-y:1rem;--bs-modal-header-padding:1rem 1rem;--bs-modal-header-border-color:var(--bs-border-color);--bs-modal-header-border-width:var(--bs-border-width);--bs-modal-title-line-height:1.5;--bs-modal-footer-gap:0.5rem;--bs-modal-footer-bg: ;--bs-modal-footer-border-color:var(--bs-border-color);--bs-modal-footer-border-width:var(--bs-border-width);z-index:var(--bs-modal-zindex);width:100%;height:100%;overflow-x:hidden;overflow-y:auto;outline:0}.input-group,.modal-body,.modal-content,.modal-dialog,.navbar{position:relative}.fade{transition:opacity .15s linear}.modal.fade{background-color:var(--bs-secondary-color)}.modal.fade .modal-dialog{transition:transform .3s ease-out}.modal-dialog{width:auto;margin:var(--bs-modal-margin);pointer-events:none}.modal-dialog-centered{display:flex;align-items:center;min-height:calc(100% - var(--bs-modal-margin) * 2)}.modal-dialog-scrollable{height:calc(100% - var(--bs-modal-margin) * 2)}.modal-dialog-scrollable .modal-content{max-height:100%;overflow:hidden}.modal-content{display:flex;flex-direction:column;width:100%;color:var(--bs-modal-color);pointer-events:auto;background-color:var(--bs-modal-bg);background-clip:padding-box;border:var(--bs-modal-border-width) solid var(--bs-modal-border-color);border-radius:var(--bs-modal-border-radius);outline:0}.modal-dialog-scrollable .modal-body{overflow-y:auto}.modal-body{flex:1 1 auto;padding:var(--bs-modal-padding)}.modal-header{display:flex;flex-shrink:0;align-items:center;justify-content:space-between;padding:var(--bs-modal-header-padding);border-bottom:var(--bs-modal-header-border-width) solid var(--bs-modal-header-border-color);border-top-left-radius:var(--bs-modal-inner-border-radius);border-top-right-radius:var(--bs-modal-inner-border-radius)}.btn-close{--bs-btn-close-color:#000;/* corrupted join                                                                                                                      */--bs-btn-close-bg:url("data:image/svg+xml,%3csvg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 16 16' fill='%23000'%3e%3cpath d='M.293.293a1 1 0 0 1 1.414 0L8 6.586 14.293.293a1 1 0 1 1 1.414 1.414L9.414 8l6.293 6.293a1 1 0 0 1-1.414 1.414L8 9.414l-6.293 6.293a1 1 0 0 1-1.414-1.414L6.586 8 .293 1.707a1 1 0 0 1 0-1.414z'/%3e%3c/svg%3e");--bs-btn-close-opacity:0.5;--bs-btn-close-hover-opacity:0.75;--bs-btn-close-focus-shadow:0 0 0 0.25rem rgba(13, 110, 253, 0.25);--bs-btn-close-focus-opacity:1;--bs-btn-close-disabled-opacity:0.25;--bs-btn-close-white-filter:invert(1) grayscale(100%) brightness(200%);box-sizing:content-box;width:1em;height:1em;padding:.25em;color:var(--bs-btn-close-color);background:transparent var(--bs-btn-close-bg) center/1em auto no-repeat;border:0;border-radius:.375rem;opacity:var(--bs-btn-close-opacity)}.w-100,ul{width:100%!important}.navbar{--bs-navbar-padding-x:0;--bs-navbar-padding-y:0.5rem;--bs-navbar-color:rgba(var(--bs-emphasis-color-rgb), 0.65);--bs-navbar-hover-color:rgba(var(--bs-emphasis-color-rgb), 0.8);--bs-navbar-disabled-color:rgba(var(--bs-emphasis-color-rgb), 0.3);--bs-navbar-active-color:rgba(var(--bs-emphasis-color-rgb), 1);--bs-navbar-brand-padding-y:0.3125rem;--bs-navbar-brand-margin-end:1rem;--bs-navbar-brand-font-size:1.25rem;--bs-navbar-brand-color:rgba(var(--bs-emphasis-color-rgb), 1);--bs-navbar-brand-hover-color:rgba(var(--bs-emphasis-color-rgb), 1);--bs-navbar-nav-link-padding-x:0.5rem;--bs-navbar-toggler-padding-y:0.25rem;--bs-navbar-toggler-padding-x:0.75rem;--bs-navbar-toggler-font-size:1.25rem;--bs-navbar-toggler-icon-bg:url(data:image/svg+xml,%3csvg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 30 30'%3e%3cpath stroke='rgba%2833, 37, 41, 0.75%29' stroke-linecap='round' stroke-miterlimit='10' stroke-width='2' d='M4 7h22M4 15h22M4 23h22'/%3e%3c/svg%3e);--bs-navbar-toggler-border-color:rgba(var(--bs-emphasis-color-rgb), 0.15);--bs-navbar-toggler-border-radius:var(--bs-border-radius);--bs-navbar-toggler-focus-width:0.25rem;--bs-navbar-toggler-transition:box-shadow 0.15s ease-in-out;display:flex;flex-wrap:wrap;align-items:center;justify-content:space-between;padding:var(--bs-navbar-padding-y) var(--bs-navbar-padding-x)}.form-control,.input-group-text{padding:.375rem .75rem;font-size:1rem;font-weight:400;line-height:1.5}nav{display:block}.text-end{text-align:right!important}.justify-content-end{justify-content:flex-end!important}*,::after,::before{box-sizing:border-box}ul{list-style-type:none}.border-bottom{border-bottom:var(--bs-border-width) var(--bs-border-style) var(--bs-border-color)!important}.d-flex{display:flex!important}li{display:list-item;text-align:-webkit-match-parent}.text-center{text-align:center!important}.input-group{display:flex;flex-wrap:wrap;align-items:stretch;width:100%}.input-group-text{background-color:rgba(1,3,111,.6);color:#fff;display:flex;align-items:center;color:var(--bs-body-color);text-align:center;white-space:nowrap;background-color:var(--bs-tertiary-bg);border:var(--bs-border-width) solid var(--bs-border-color);border-radius:var(--bs-border-radius)}.col-2{flex:0 0 auto;width:16.66666667%}.input-group>:not(:first-child):not(.dropdown-menu):not(.valid-tooltip):not(.valid-feedback):not(.invalid-tooltip):not(.invalid-feedback){margin-left:calc(var(--bs-border-width) * -1);border-top-left-radius:0;border-bottom-left-radius:0}.input-group:not(.has-validation)>.dropdown-toggle:nth-last-child(n+3),.input-group:not(.has-validation)>.form-floating:not(:last-child)>.form-control,.input-group:not(.has-validation)>.form-floating:not(:last-child)>.form-select,.input-group:not(.has-validation)>:not(:last-child):not(.dropdown-toggle):not(.dropdown-menu):not(.form-floating){border-top-right-radius:0;border-bottom-right-radius:0}.input-group>.form-floating:not(:first-child)>.form-control,.input-group>.form-floating:not(:first-child)>.form-select{border-top-left-radius:0;border-bottom-left-radius:0}.input-group>.form-control,.input-group>.form-floating,.input-group>.form-select{position:relative;flex:1 1 auto;width:1%;min-width:0}.form-control{display:block;width:100%;color:var(--bs-body-color);-webkit-appearance:none;-moz-appearance:none;appearance:none;background-clip:padding-box;border:var(--bs-border-width) solid var(--bs-border-color);border-radius:var(--bs-border-radius);transition:border-color .15s ease-in-out,box-shadow .15s ease-in-out}.form-control[type=file]{overflow:hidden}.fs-5{font-size:1.25rem!important}.px-5{padding-right:3rem!important;padding-left:3rem!important}.me-4{margin-right:1.5rem!important}.btn-primary{--bs-btn-color:#fff;--bs-btn-bg:#0d6efd;--bs-btn-border-color:#0d6efd;--bs-btn-hover-color:#fff;--bs-btn-hover-bg:#0b5ed7;--bs-btn-hover-border-color:#0a58ca;--bs-btn-focus-shadow-rgb:49,132,253;--bs-btn-active-color:#fff;--bs-btn-active-bg:#0a58ca;--bs-btn-active-border-color:#0a53be;--bs-btn-active-shadow:inset 0 3px 5px rgba(0, 0, 0, 0.125);--bs-btn-disabled-color:#fff;--bs-btn-disabled-bg:#0d6efd;--bs-btn-disabled-border-color:#0d6efd}.btn{--bs-btn-padding-x:0.75rem;--bs-btn-padding-y:0.375rem;--bs-btn-border-radius:var(--bs-border-radius);padding:var(--bs-btn-padding-y) var(--bs-btn-padding-x);font-family:var(--bs-btn-font-family);font-size:var(--bs-btn-font-size);font-weight:var(--bs-btn-font-weight);line-height:var(--bs-btn-line-height);color:var(--bs-btn-color);text-align:center;text-decoration:none;vertical-align:middle;-webkit-user-select:none;-moz-user-select:none;user-select:none;border:var(--bs-btn-border-width) solid var(--bs-btn-border-color);border-radius:var(--bs-btn-border-radius);background-color:var(--bs-btn-bg);/*   corrupted join                                                                                                                                                                                                  */transition:color .15s ease-in-out,background-color .15s ease-in-out,border-color .15s ease-in-out,box-shadow .15s ease-in-out}[type=button],[type=reset],[type=submit],button{-webkit-appearance:button}button,input,optgroup,select,textarea{margin:0;font-family:inherit;font-size:inherit;line-height:inherit}input[type=text i]{writing-mode:horizontal-tb!important;padding-block:1px;padding-inline:2px}input{text-rendering:auto;color:fieldtext;letter-spacing:normal;word-spacing:normal;line-height:normal;text-indent:0;text-shadow:none;display:inline-block;text-align:start;appearance:auto;-webkit-rtl-ordering:logical;cursor:text;background-color:field;padding:1px 0;border-width:2px;border-style:inset;border-color:-internal-light-dark(#767676,#858585);border-image:initial;padding-block:1px;padding-inline:2px}input:focus{box-shadow:0 0 0 .2rem var(--bs-focus-ring-color)}.form-floating>.form-control-plaintext~label,.form-floating>.form-control:focus~label,.form-floating>.form-control:not(:placeholder-shown)~label,.form-floating>.form-select~label{color:rgba(var(--bs-body-color-rgb),.65);transform:scale(.85) translateY(-.5rem) translateX(.15rem)}.form-floating>label{position:absolute;top:0;left:0;z-index:2;height:100%;padding:1rem .75rem;overflow:hidden;text-align:start;text-overflow:ellipsis;white-space:nowrap;pointer-events:none;border:var(--bs-border-width) solid transparent;transform-origin:0 0;transition:opacity .1s ease-in-out,transform .1s ease-in-out}.form-floating>.form-select{padding-top:1.625rem;padding-bottom:.625rem}.form-floating>.form-control,.form-floating>.form-control-plaintext,.form-floating>.form-select{height:calc(3.5rem + calc(var(--bs-border-width) * 2));min-height:calc(3.5rem + calc(var(--bs-border-width) * 2));line-height:1.25}.form-select{--bs-form-select-bg-img:url(data:image/svg+xml,%3csvg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 16 16'%3e%3cpath fill='none' stroke='%23343a40' stroke-linecap='round' stroke-linejoin='round' stroke-width='2' d='m2 5 6 6 6-6'/%3e%3c/svg%3e);display:block;width:100%;padding:.375rem 2.25rem .375rem .75rem;font-size:1rem;font-weight:400;line-height:1.5;color:var(--bs-body-color);-webkit-appearance:none;-moz-appearance:none;appearance:none;background-image:var(--bs-form-select-bg-img),var(--bs-form-select-bg-icon,none);background-repeat:no-repeat;background-position:right .75rem center;background-size:16px 12px;border:var(--bs-border-width) solid var(--bs-border-color);border-radius:var(--bs-border-radius);transition:border-color .15s ease-in-out,box-shadow .15s ease-in-out}select{word-wrap:normal}.whiteF::before{filter:invert(99%) sepia(0%) saturate(617%) hue-rotate(237deg) brightness(109%) contrast(98%)}.bi-save::before{content:url('data:image/svg+xml;utf8,<svg xmlns="http://www.w3.org/2000/svg" width="16" fill="currentColor" class="bi bi-floppy" viewBox="0 0 16 16"><path d="M11 2H9v3h2z"></path><path d="M1.5 0h11.586a1.5 1.5 0 0 1 1.06.44l1.415 1.414A1.5 1.5 0 0 1 16 2.914V14.5a1.5 1.5 0 0 1-1.5 1.5h-13A1.5 1.5 0 0 1 0 14.5v-13A1.5 1.5 0 0 1 1.5 0M1 1.5v13a.5.5 0 0 0 .5.5H2v-4.5A1.5 1.5 0 0 1 3.5 9h9a1.5 1.5 0 0 1 1.5 1.5V15h.5a.5.5 0 0 0 .5-.5V2.914a.5.5 0 0 0-.146-.353l-1.415-1.415A.5.5 0 0 0 13.086 1H13v4.5A1.5 1.5 0 0 1 11.5 7h-7A1.5 1.5 0 0 1 3 5.5V1H1.5a.5.5 0 0 0-.5.5m3 4a.5.5 0 0 0 .5.5h7a.5.5 0 0 0 .5-.5V1H4zM3 15h10v-4.5a.5.5 0 0 0-.5-.5h-9a.5.5 0 0 0-.5.5z"></path></svg>')}.bi-trash3::before{content:url('data:image/svg+xml;utf8,<svg xmlns="http://www.w3.org/2000/svg" width="16" height="16" fill="currentColor" class="bi bi-trash3" viewBox="0 0 16 16"><path d="M6.5 1h3a.5.5 0 0 1 .5.5v1H6v-1a.5.5 0 0 1 .5-.5M11 2.5v-1A1.5 1.5 0 0 0 9.5 0h-3A1.5 1.5 0 0 0 5 1.5v1H1.5a.5.5 0 0 0 0 1h.538l.853 10.66A2 2 0 0 0 4.885 16h6.23a2 2 0 0 0 1.994-1.84l.853-10.66h.538a.5.5 0 0 0 0-1zm1.958 1-.846 10.58a1 1 0 0 1-.997.92h-6.23a1 1 0 0 1-.997-.92L3.042 3.5zm-7.487 1a.5.5 0 0 1 .528.47l.5 8.5a.5.5 0 0 1-.998.06L5 5.03a.5.5 0 0 1 .47-.53Zm5.058 0a.5.5 0 0 1 .47.53l-.5 8.5a.5.5 0 1 1-.998-.06l.5-8.5a.5.5 0 0 1 .528-.47M8 4.5a.5.5 0 0 1 .5.5v8.5a.5.5 0 0 1-1 0V5a.5.5 0 0 1 .5-.5"/></svg>')}.bi-search::before{content:url('data:image/svg+xml;utf8,<svg xmlns="http://www.w3.org/2000/svg" width="16" height="16" fill="currentColor" class="bi bi-search" viewBox="0 0 16 16"><path d="M11.742 10.344a6.5 6.5 0 1 0-1.397 1.398h-.001q.044.06.098.115l3.85 3.85a1 1 0 0 0 1.415-1.414l-3.85-3.85a1 1 0 0 0-.115-.1zM12 6.5a5.5 5.5 0 1 1-11 0 5.5 5.5 0 0 1 11 0"/></svg>')}.bi-file-earmark-medical::before{content:url('data:image/svg+xml;utf8,<svg xmlns="http://www.w3.org/2000/svg" width="16" height="16" fill="currentColor" class="bi bi-file-earmark-medical" viewBox="0 0 16 16"><path d="M7.5 5.5a.5.5 0 0 0-1 0v.634l-.549-.317a.5.5 0 1 0-.5.866L6 7l-.549.317a.5.5 0 1 0 .5.866l.549-.317V8.5a.5.5 0 1 0 1 0v-.634l.549.317a.5.5 0 1 0 .5-.866L8 7l.549-.317a.5.5 0 1 0-.5-.866l-.549.317zm-2 4.5a.5.5 0 0 0 0 1h5a.5.5 0 0 0 0-1zm0 2a.5.5 0 0 0 0 1h5a.5.5 0 0 0 0-1z"/><path d="M14 14V4.5L9.5 0H4a2 2 0 0 0-2 2v12a2 2 0 0 0 2 2h8a2 2 0 0 0 2-2M9.5 3A1.5 1.5 0 0 0 11 4.5h2V14a1 1 0 0 1-1 1H4a1 1 0 0 1-1-1V2a1 1 0 0 1 1-1h5.5z"/></svg>')}.bi-file-earmark-plus::before{content:url('data:image/svg+xml;utf8,<svg xmlns="http://www.w3.org/2000/svg" width="16" height="16" fill="currentColor" class="bi bi-file-earmark-plus" viewBox="0 0 16 16"><path d="M8 6.5a.5.5 0 0 1 .5.5v1.5H10a.5.5 0 0 1 0 1H8.5V11a.5.5 0 0 1-1 0V9.5H6a.5.5 0 0 1 0-1h1.5V7a.5.5 0 0 1 .5-.5"/><path d="M14 4.5V14a2 2 0 0 1-2 2H4a2 2 0 0 1-2-2V2a2 2 0 0 1 2-2h5.5zm-3 0A1.5 1.5 0 0 1 9.5 3V1H4a1 1 0 0 0-1 1v12a1 1 0 0 0 1 1h8a1 1 0 0 0 1-1V4.5z"/></svg>')}.bi-router::before{content:url('data:image/svg+xml;utf8,<svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" fill="currentColor" class="bi bi-router" viewBox="0 0 16 16"><path d="M5.525 3.025a3.5 3.5 0 0 1 4.95 0 .5.5 0 1 0 .707-.707 4.5 4.5 0 0 0-6.364 0 .5.5 0 0 0 .707.707"/><path d="M6.94 4.44a1.5 1.5 0 0 1 2.12 0 .5.5 0 0 0 .708-.708 2.5 2.5 0 0 0-3.536 0 .5.5 0 0 0 .707.707ZM2.5 11a.5.5 0 1 1 0-1 .5.5 0 0 1 0 1m4.5-.5a.5.5 0 1 0 1 0 .5.5 0 0 0-1 0m2.5.5a.5.5 0 1 1 0-1 .5.5 0 0 1 0 1m1.5-.5a.5.5 0 1 0 1 0 .5.5 0 0 0-1 0m2 0a.5.5 0 1 0 1 0 .5.5 0 0 0-1 0"/><path d="M2.974 2.342a.5.5 0 1 0-.948.316L3.806 8H1.5A1.5 1.5 0 0 0 0 9.5v2A1.5 1.5 0 0 0 1.5 13H2a.5.5 0 0 0 .5.5h2A.5.5 0 0 0 5 13h6a.5.5 0 0 0 .5.5h2a.5.5 0 0 0 .5-.5h.5a1.5 1.5 0 0 0 1.5-1.5v-2A1.5 1.5 0 0 0 14.5 8h-2.306l1.78-5.342a.5.5 0 1 0-.948-.316L11.14 8H4.86zM14.5 9a.5.5 0 0 1 .5.5v2a.5.5 0 0 1-.5.5h-13a.5.5 0 0 1-.5-.5v-2a.5.5 0 0 1 .5-.5z"/><path d="M8.5 5.5a.5.5 0 1 1-1 0 .5.5 0 0 1 1 0"/></svg>')}.bi-stopwatch::before{content:url('data:image/svg+xml;utf8,<svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" fill="currentColor" class="bi bi-stopwatch" viewBox="0 0 16 16"><path d="M8.5 5.6a.5.5 0 1 0-1 0v2.9h-3a.5.5 0 0 0 0 1H8a.5.5 0 0 0 .5-.5z"/><path d="M6.5 1A.5.5 0 0 1 7 .5h2a.5.5 0 0 1 0 1v.57c1.36.196 2.594.78 3.584 1.64l.012-.013.354-.354-.354-.353a.5.5 0 0 1 .707-.708l1.414 1.415a.5.5 0 1 1-.707.707l-.353-.354-.354.354-.013.012A7 7 0 1 1 7 2.071V1.5a.5.5 0 0 1-.5-.5M8 3a6 6 0 1 0 .001 12A6 6 0 0 0 8 3"/></svg>')}.bi-puzzle::before{content:url('data:image/svg+xml;utf8,<svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" fill="currentColor" class="bi bi-puzzle" viewBox="0 0 16 16"><path d="M3.112 3.645A1.5 1.5 0 0 1 4.605 2H7a.5.5 0 0 1 .5.5v.382c0 .696-.497 1.182-.872 1.469a.5.5 0 0 0-.115.118l-.012.025L6.5 4.5v.003l.003.01q.005.015.036.053a.9.9 0 0 0 .27.194C7.09 4.9 7.51 5 8 5c.492 0 .912-.1 1.19-.24a.9.9 0 0 0 .271-.194.2.2 0 0 0 .039-.063v-.009l-.012-.025a.5.5 0 0 0-.115-.118c-.375-.287-.872-.773-.872-1.469V2.5A.5.5 0 0 1 9 2h2.395a1.5 1.5 0 0 1 1.493 1.645L12.645 6.5h.237c.195 0 .42-.147.675-.48.21-.274.528-.52.943-.52.568 0 .947.447 1.154.862C15.877 6.807 16 7.387 16 8s-.123 1.193-.346 1.638c-.207.415-.586.862-1.154.862-.415 0-.733-.246-.943-.52-.255-.333-.48-.48-.675-.48h-.237l.243 2.855A1.5 1.5 0 0 1 11.395 14H9a.5.5 0 0 1-.5-.5v-.382c0-.696.497-1.182.872-1.469a.5.5 0 0 0 .115-.118l.012-.025.001-.006v-.003a.2.2 0 0 0-.039-.064.9.9 0 0 0-.27-.193C8.91 11.1 8.49 11 8 11s-.912.1-1.19.24a.9.9 0 0 0-.271.194.2.2 0 0 0-.039.063v.003l.001.006.012.025c.016.027.05.068.115.118.375.287.872.773.872 1.469v.382a.5.5 0 0 1-.5.5H4.605a1.5 1.5 0 0 1-1.493-1.645L3.356 9.5h-.238c-.195 0-.42.147-.675.48-.21.274-.528.52-.943.52-.568 0-.947-.447-1.154-.862C.123 9.193 0 8.613 0 8s.123-1.193.346-1.638C.553 5.947.932 5.5 1.5 5.5c.415 0 .733.246.943.52.255.333.48.48.675.48h.238zM4.605 3a.5.5 0 0 0-.498.55l.001.007.29 3.4A.5.5 0 0 1 3.9 7.5h-.782c-.696 0-1.182-.497-1.469-.872a.5.5 0 0 0-.118-.115l-.025-.012L1.5 6.5h-.003a.2.2 0 0 0-.064.039.9.9 0 0 0-.193.27C1.1 7.09 1 7.51 1 8s.1.912.24 1.19c.07.14.14.225.194.271a.2.2 0 0 0 .063.039H1.5l.006-.001.025-.012a.5.5 0 0 0 .118-.115c.287-.375.773-.872 1.469-.872H3.9a.5.5 0 0 1 .498.542l-.29 3.408a.5.5 0 0 0 .497.55h1.878c-.048-.166-.195-.352-.463-.557-.274-.21-.52-.528-.52-.943 0-.568.447-.947.862-1.154C6.807 10.123 7.387 10 8 10s1.193.123 1.638.346c.415.207.862.586.862 1.154 0 .415-.246.733-.52.943-.268.205-.415.39-.463.557h1.878a.5.5 0 0 0 .498-.55l-.001-.007-.29-3.4A.5.5 0 0 1 12.1 8.5h.782c.696 0 1.182.497 1.469.872.05.065.091.099.118.115l.025.012.006.001h.003a.2.2 0 0 0 .064-.039.9.9 0 0 0 .193-.27c.14-.28.24-.7.24-1.191s-.1-.912-.24-1.19a.9.9 0 0 0-.194-.271.2.2 0 0 0-.063-.039H14.5l-.006.001-.025.012a.5.5 0 0 0-.118.115c-.287.375-.773.872-1.469.872H12.1a.5.5 0 0 1-.498-.543l.29-3.407a.5.5 0 0 0-.497-.55H9.517c.048.166.195.352.463.557.274.21.52.528.52.943 0 .568-.447.947-.862 1.154C9.193 5.877 8.613 6 8 6s-1.193-.123-1.638-.346C5.947 5.447 5.5 5.068 5.5 4.5c0-.415.246-.733.52-.943.268-.205.415-.39.463-.557z"/></svg>')}.bi-folder2-open::before{content:url('data:image/svg+xml;utf8,<svg xmlns="http://www.w3.org/2000/svg" width="16" height="16" fill="currentColor" class="bi bi-folder2-open" viewBox="0 0 16 16"><path d="M1 3.5A1.5 1.5 0 0 1 2.5 2h2.764c.958 0 1.76.56 2.311 1.184C7.985 3.648 8.48 4 9 4h4.5A1.5 1.5 0 0 1 15 5.5v.64c.57.265.94.876.856 1.546l-.64 5.124A2.5 2.5 0 0 1 12.733 15H3.266a2.5 2.5 0 0 1-2.481-2.19l-.64-5.124A1.5 1.5 0 0 1 1 6.14zM2 6h12v-.5a.5.5 0 0 0-.5-.5H9c-.964 0-1.71-.629-2.174-1.154C6.374 3.334 5.82 3 5.264 3H2.5a.5.5 0 0 0-.5.5zm-.367 1a.5.5 0 0 0-.496.562l.64 5.124A1.5 1.5 0 0 0 3.266 14h9.468a1.5 1.5 0 0 0 1.489-1.314l.64-5.124A.5.5 0 0 0 14.367 7z"/></svg>')}.bi-power::before{content:url('data:image/svg+xml;utf8,<svg xmlns="http://www.w3.org/2000/svg" width="16" height="16" fill="currentColor" class="bi bi-power" viewBox="0 0 16 16"><path d="M7.5 1v7h1V1z"/><path d="M3 8.812a5 5 0 0 1 2.578-4.375l-.485-.874A6 6 0 1 0 11 3.616l-.501.865A5 5 0 1 1 3 8.812"/></svg>')}.bi::before,[class*=" bi-"]::before,[class^=bi-]::before{display:inline-block;font-family:bootstrap-icons!important;font-style:normal;font-weight:400!important;font-variant:normal;text-transform:none;line-height:1;vertical-align:-.125em;-webkit-font-smoothing:antialiased;-moz-osx-font-smoothing:grayscale}.flex-fill {flex: 1 1 auto!important;}.me-auto {margin-right: auto!important;}
    .facts::before {
      content: "";
      position: absolute;
      left: 0;
      right: 0;
      top: 0;
      bottom: 0;
      background: rgba(1, 3, 111, 0.6);
    }
    .input-group-text {
      background-color: rgba(1, 3, 111, 0.6);
      color: #ffffff;
    }
    .nav h1 {
      font-weight: 800;
      position: absolute!important;
      left: 0!important;
      top: 0!important;
      margin: 0px 1rem;
      font-size: 2em;
    }
    p {
      color: rgba(255, 255, 255, 0.6);
      position: relative;
    }
    /* corrupted join workaround          */
    .facts h2{
      margin-bottom: 0.5rem;
      color: #ffffff;
      position: relative;
    }
  </style>
</head>
<body>
  <nav class="navbar fixed-top py-0 navbar-light text-light">
    <ul class="nav w-100 justify-content-end list-unstyled d-flex">
      <h1>Board Configuration</h1>
      <li class="text-end text-muted size-ico me-4" role="button">Save <span class="btn-outline-primary size-ico btn-submit bi-save"></span></li>
      <li class="text-end text-muted size-ico me-4 modal-button-files" role="button">Files <span class="btn-outline-primary size-ico bi-folder2-open"></span></li>
      <li class="text-end text-muted size-ico me-4 modal-button-power" role="button">Reboot <span class="size-ico bi-power"></span></li>
    </ul>
  </nav>

  <div class="facts">
    <div class="input-group mb-3">
      <span class="input-group-text col-2">Steer Settings File</span>
      <input class="form-control" id="steerSettingsFile" type="text" value="/steerSettings.json">
    </div>

    <div class="input-group mb-3">
      <span class="input-group-text col-2">Steer Configuration File</span>
      <input class="form-control" id="steerConfigurationFile" type="text" value="/steerConfiguration.json">
    </div>

    <h2 class="size-ico bi-router whiteF"> Network</h2>
    <div class="input-group">
      <span class="input-group-text col-2">Board Ip</span>
      <input type="text" id="eth-ip0" value="192" class="form-control">
      <input type="text" id="eth-ip1" value="168" class="form-control">
      <input type="text" id="eth-ip2" value="1" class="form-control">
      <input type="text" id="eth-ip3" value="123" class="form-control">
    </div>

    <div class="input-group">
      <span class="input-group-text col-2">Gateway</span>
      <input type="text" id="eth-gateway0" value="192" class="form-control">
      <input type="text" id="eth-gateway1" value="168" class="form-control">
      <input type="text" id="eth-gateway2" value="1" class="form-control">
      <input type="text" id="eth-gateway3" value="1" class="form-control">
    </div>

    <div class="input-group">
      <span class="input-group-text col-2">Subnet</span>
      <input type="text" id="eth-subnet0" value="255" class="form-control">
      <input type="text" id="eth-subnet1" value="255" class="form-control">
      <input type="text" id="eth-subnet2" value="255" class="form-control">
      <input type="text" id="eth-subnet3" value="0" class="form-control">
    </div>

    <div class="input-group">
      <span class="input-group-text col-2">Dns</span>
      <input type="text" id="eth-dns0" value="8" class="form-control">
      <input type="text" id="eth-dns1" value="8" class="form-control">
      <input type="text" id="eth-dns2" value="8" class="form-control">
      <input type="text" id="eth-dns3" value="8" class="form-control">
    </div>

    <div class="input-group pt-3">
      <span class="input-group-text col-2">Server Ip</span>
      <input type="text" id="server-ip0" value="192" class="form-control">
      <input type="text" id="server-ip1" value="168" class="form-control">
      <input type="text" id="server-ip2" value="1" class="form-control">
      <input type="text" id="server-ip3" value="255" class="form-control">
    </div>

    <div class="input-group pb-3">
      <span class="input-group-text col-2">UDP Ports</span>
      <div class="form-floating col-3">
        <input class="form-control" id="pcbPort" type="text" value="5120">
        <label for="pcbPort">Board Port</label>
      </div>
      <div class="form-floating col-3">
        <input class="form-control" id="ntripPort" type="text" value="2233">
        <label for="ntripPort">NTRIP Port</label>
      </div>
      <div class="form-floating col-3">
        <input class="form-control" id="autosteerPort" type="text" value="8888">
        <label for="autosteerPort">Autosteer Port</label>
      </div>
      <div class="form-floating col-3">
        <input class="form-control" id="destinationPort" type="text" value="9999">
        <label for="destinationPort">Destination Port</label>
      </div>
    </div>

    <h2 class="size-ico bi-puzzle whiteF"> Components</h2>
    <div class="input-group mb-3">
      <label class="input-group-text col-2">Vehicle CAN</label>
      <div class="form-floating">
        <select class="form-select" id="can-brand">
          <option value="0" selected>Class (1E/30 Navigation Controller, 13/19 Steering Controller)</option>
          <option value="1">Valtra, Massey Ferguson (Standard Danfoss ISO 1C/28 Navigation Controller, 13/19 Steering Controller)</option>
          <option value="2">CaseIH, New Holland (AA/170 Navagation Controller, 08/08 Steering Controller)</option>
          <option value="3">Fendt (2C/44 Navigation Controller, F0/240 Steering Controller)</option>
          <option value="4">JCB (AB/171 Navigation Controller, 13/19 Steering Controller)</option>
          <option value="5">FendtOne</option>
          <option value="6">Lindner (F0/240 Navigation Controller, 13/19 Steering Controller)</option>
          <option value="7">AgOpenGPS - Remote CAN/PWM module (1C/28 Navigation Controller, 13/19 Steering Controller)</option>
        </select>
        <label for="can-brand">Brand</label>
      </div>
      <div class="form-floating">
        <select class="form-select" id="can-mode">
          <option value="0" selected>None</option>
          <option value="1">V & ISO & K - (No Keya)</option>
          <option value="2">V & ISO & Keya - (No K)</option>
          <option value="3">V & Keya & K - (No ISO)</option>
          <option value="4">V & ISO - (No Keya, No K)</option>
          <option value="5">V & K - (No ISO, No Keya)</option>
          <option value="6">V & Keya - (No ISO, No K)</option>
          <!-- corrupted join workaround --><option value="7">V - (No ISO, No Keya, No K)</option>
          <option value="8"> Keya - (No V, No ISO, No K)</option>
        </select>
        <label for="can-mode">Mode</label>
      </div>
    </div>
    <div class="input-group mb-3">
      <label class="input-group-text col-2" for="driver">Driver</label>
      <div class="form-floating">
        <select class="form-select" id="driver">
          <option value="1" selected>Cytron MD13s</option>
          <option value="2">Keya CAN</option>
          <option value="3">IBT-2</option>
        </select>
        <label for="driver">Type</label>
      </div>
      <div class="form-floating">
        <input class="form-control" id="driver-pin0" type="text" value="0">
        <label for="driver-pin0">Dir</label>
      </div>
      <div class="form-floating">
        <input class="form-control" id="driver-pin1" type="text" value="4">
        <label for="driver-pin1">PWM</label>
      </div>
      <div class="form-floating">
        <input class="form-control" id="driver-pin2" type="text" value="12">
        <label for="driver-pin2">NC</label>
      </div>
    </div>
    <div class="input-group mb-3">
      <label class="input-group-text col-2">GNSS</label>
      <div class="form-floating">
        <input class="form-control" id="gnss-port" type="text" value="2">
        <label for="gnss-port">Port</label>
      </div>
      <div class="form-floating">
        <input class="form-control" id="gnss-baudRate" type="text" value="460800">
        <label for="gnss-baudRate">BaudRate</label>
      </div>
    </div>

    <div class="input-group mb-3">
      <label class="input-group-text col-2" for="imu">IMU</label>
      <div class="form-floating">
        <select class="form-select" id="imu">
          <option value="1" selected>Serial (RVC)</option>
          <option value="2">Petition based</option>
        </select>
        <label for="imu">Type</label>
      </div>
      <div class="form-floating">
        <select class="form-select" id="imu-axisOrder">
          <option value="1" selected>XYZ</option>
          <option value="2">YXZ</option>
          <option value="3">ZXY</option>
          <option value="4">ZYX</option>
          <option value="5">YZX</option>
          <option value="6">XZY</option>
        </select>
        <label for="imu-axisOrder">Axis Orientation</label>
      </div>
      <div class="form-floating">
        <input class="form-control" id="imu-port" type="text" value="1">
        <label for="imu-port">Port</label>
      </div>
      <div class="form-floating">
        <input class="form-control" id="imu-tickRate" type="text" value="11000">
        <label for="imu-tickRate">Tick Rate (mHz)</label>
      </div>
    </div>

    <div class="input-group mb-3">
      <label class="input-group-text col-2" for="was">WAS</label>
      <div class="form-floating">
        <select class="form-select" id="was">
          <option value="1" selected>Internal</option>
          <option value="2">ADS1115</option>
          <option value="3">Vehicle CAN (needs correct can mode value)</option>
        </select>
        <label for="was">Type</label>
      </div>
      <div class="form-floating">
        <input class="form-control" id="was-resolution" type="text" value="12">
        <label for="was-resolution">Resolution</label>
      </div>
      <div class="form-floating">
        <input class="form-control" id="was-pin" type="text" value="35">
        <label for="was-pin">Pin</label>
      </div>
    </div>

    <div class="input-group mb-3">
      <label class="input-group-text col-2">Load Sensing</label>
      <div class="form-floating">
        <input class="form-control" id="ls-pin" type="text" value="39">
        <label for="ls-pin">Pin</label>
      </div>
      <div class="form-floating">
        <input class="form-control" id="ls-filter" type="text" value="2">
        <label for="ls-filter">Filter</label>
      </div>
    </div>

    <div class="input-group mb-3">
      <label class="input-group-text col-2">Buttons pins</label>
      <div class="form-floating">
        <input class="form-control" id="remotePin" type="text" value="39">
        <label for="remotePin">Remote</label>
      </div>
      <div class="form-floating">
        <input class="form-control" id="steerPin" type="text" value="36">
        <label for="steerPin">Steer</label>
      </div>
      <div class="form-floating">
        <input class="form-control" id="workPin" type="text" value="1">
        <label for="workPin">Work</label>
      </div>
    </div>

    <h2 class="size-ico bi-stopwatch whiteF"> Timing</h2>
    <div class="input-group mb-3">
      <label class="input-group-text col-2">Timers</label>
      <div class="form-floating">
        <input class="form-control" id="reportTickRate" type="text" value="10000">
        <label for="reportTickRate">Report Tick Rate (mHz)</label>
      </div>
      <div class="form-floating">
        <input class="form-control" id="globalTickRate" type="text" value="10000">
        <label for="globalTickRate">Gobal Tick Rate (mHz)</label>
      </div>
    </div>

    <div class="w-100 justify-content-end" style="position: relative; margin-top:1rem;">
      <button type="button" class="btn btn-primary btn-submit bi-save whiteF fs-5 px-5"> Save</button>
    </div>
  </div>

  <!-- Modal Files -->
  <div class="modal fade" id="filesModal" tabindex="-1" style="display: none;">
    <div class="modal-dialog modal-dialog-centered modal-dialog-scrollable modal-xl">
      <div class="modal-content">
        <div class="modal-header">
          <h2>Files stored</h2>
          <button type="button" class="btn-close" data-bs-dismiss="modal" aria-label="Close"></button>
        </div>
        <div class="modal-body d-flex"><!--                  Corrupted join workaround                 -->
          <div class="card-body w-100" id="fs-list">
            <button type="button" class="btn btn-primary w-100 bi-file-earmark-plus file-add whiteF fs-5 px-5"> Add file</button>
            <div id="status"></div>
            <div id="detailsheader"></div>
            <div id="details"></div>
          </div>
        </div>
      </div>
    </div>
  </div>
  <!-- Modal Power -->
  <div class="modal fade" id="rebootModal" tabindex="-1" style="display: none;">
    <div class="modal-dialog modal-dialog-centered modal-dialog-scrollable">
      <div class="modal-content">
        <div class="modal-header">
          <h2>Confirm Reboot</h2>
          <button type="button" class="btn-close" data-bs-dismiss="modal" aria-label="Close"></button>
        </div>
        <div class="modal-body d-flex w-100">
          <span class="btn btn-primary w-100 bi-power whiteF fs-6 mt-3" id="reboot"> Reboot</span>
        </div>
      </div>
    </div>
  </div>
  <div id="preloader"></div>
  <!-- Template Main JS File -->
  <script type="text/javascript">
    (function() {
      "use strict";

      /**
       * Form helper
       */
      const val = (el, txt=false) => {
        el = el.trim()
        if (txt) {
          return document.querySelector(el).value;
        } else {
          let v = Number(document.querySelector(el).value);
          return isNaN(v)? 0 : v;
        }
      }

      document.querySelectorAll('.btn-submit').forEach(e=>e.addEventListener('click',()=>{
        let data={steerSettingsFile:val("#steerSettingsFile",true), steerConfigurationFile:val("#steerConfigurationFile",true),
                  eth:{
                    ip:[val("#eth-ip0"),val("#eth-ip1"),val("#eth-ip2"),val("#eth-ip3")],
                    gateway:[val("#eth-gateway0"),val("#eth-gateway1"),val("#eth-gateway2"),val("#eth-gateway3")],
                    subnet:[val("#eth-subnet0"),val("#eth-subnet1"),val("#eth-subnet2"),val("#eth-subnet3")],
                    dns:[val("#eth-dns0"),val("#eth-dns1"),val("#eth-dns2"),val("#eth-dns3")]
                  },
                  server:{
                    ip:[val("#server-ip0"),val("#server-ip1"),val("#server-ip2"),val("#server-ip3")],
                    pcbPort:val("#pcbPort"),
                    ntripPort:val("#ntripPort"),
                    autosteerPort:val("#autosteerPort"),
                    destinationPort:val("#destinationPort")
                  },
                  can:{
                    brand:val("#can-brand"),
                    mode:val("#can-mode")
                  },
                  driver:{
                    type:val("#driver"),
                    pin:[val("#driver-pin0"),val("#driver-pin1"),val("#driver-pin2")]
                  },
                  gnss:{
                    port:val("#gnss-port"),
                    baudRate:val("#gnss-baudRate")
                  },
                  imu:{
                    type:val("#imu"),
                    port:val("#imu-port"),
                    axisOrder:val("#imu-axisOrder"),
                    tickRate:val("#imu-tickRate")
                  },
                  was:{
                    type:val("#was"),
                    resolution:val("#was-resolution"),
                    pin:val("#was-pin")
                  },
                  ls:{
                    pin:val("#ls-pin"),
                    filter:val("#ls-filter")
                  },
                  remotePin:val("#remotePin"),
                  steerPin:val("#steerPin"),
                  workPin:val("#workPin"),
                  reportTickRate:val("#reportTickRate"),
                  globalTickRate:val("#globalTickRate")
                };
        fetch('/save', {
            method: "POST",
            body: JSON.stringify(data),
            headers: {"Content-type": "application/json; charset=UTF-8"}
          })
          .then(response => response.json())
          .then(json => console.log(json))
          .catch(err => console.log(err));
      }));

      document.querySelectorAll('.file-add').forEach((el)=>{el.addEventListener('click',()=>{fileSelect();})});

      fetch('./configuration.json')
        .then((response) => response.json())
        .then((conf) =>{
          document.querySelector("#steerSettingsFile").value = conf.steerSettingsFile;
          document.querySelector("#steerConfigurationFile").value = conf.steerConfigurationFile;
          document.querySelector("#eth-ip0").value = conf.eth.ip[0];
          document.querySelector("#eth-ip1").value = conf.eth.ip[1];
          document.querySelector("#eth-ip2").value = conf.eth.ip[2];
          document.querySelector("#eth-ip3").value = conf.eth.ip[3];
          document.querySelector("#eth-gateway0").value = conf.eth.gateway[0];
          document.querySelector("#eth-gateway1").value = conf.eth.gateway[1];
          document.querySelector("#eth-gateway2").value = conf.eth.gateway[2];
          document.querySelector("#eth-gateway3").value = conf.eth.gateway[3];
          document.querySelector("#eth-subnet0").value = conf.eth.subnet[0];
          document.querySelector("#eth-subnet1").value = conf.eth.subnet[1];
          document.querySelector("#eth-subnet2").value = conf.eth.subnet[2];
          document.querySelector("#eth-subnet3").value = conf.eth.subnet[3];
          document.querySelector("#eth-dns0").value = conf.eth.dns[0];
          document.querySelector("#eth-dns1").value = conf.eth.dns[1];
          document.querySelector("#eth-dns2").value = conf.eth.dns[2];
          document.querySelector("#eth-dns3").value = conf.eth.dns[3];
          document.querySelector("#server-ip0").value = conf.server.ip[0];
          document.querySelector("#server-ip1").value = conf.server.ip[1];
          document.querySelector("#server-ip2").value = conf.server.ip[2];
          document.querySelector("#server-ip3").value = conf.server.ip[3];
          document.querySelector("#pcbPort").value = conf.server.pcbPort;
          document.querySelector("#ntripPort").value = conf.server.ntripPort;
          document.querySelector("#autosteerPort").value = conf.server.autosteerPort;
          document.querySelector("#destinationPort").value = conf.server.destinationPort;
          document.querySelector("#can-brand").value = conf.can.brand;
          document.querySelector("#can-mode").value = conf.can.mode;
          document.querySelector("#driver").value = conf.driver.type;
          document.querySelector("#driver-pin0").value = conf.driver.pin[0];
          document.querySelector("#driver-pin1").value = conf.driver.pin[1];
          document.querySelector("#driver-pin2").value = conf.driver.pin[2];
          document.querySelector("#gnss-port").value = conf.gnss.port;
          document.querySelector("#gnss-baudRate").value = conf.gnss.baudRate;
          document.querySelector("#imu").value = conf.imu.type;
          document.querySelector("#imu-port").value = conf.imu.port;
          document.querySelector("#imu-axisOrder").value = conf.imu.axisOrder;
          document.querySelector("#imu-tickRate").value = conf.imu.tickRate;
          document.querySelector("#was").value = conf.was.type;
          document.querySelector("#was-resolution").value = conf.was.resolution;
          document.querySelector("#was-pin").value = conf.was.pin;
          document.querySelector("#ls-pin").value = conf.ls.pin;
          document.querySelector("#ls-filter").value = conf.ls.filter;
          document.querySelector("#remotePin").value = conf.remotePin;
          document.querySelector("#steerPin").value = conf.steerPin;
          document.querySelector("#workPin").value = conf.workPin;
          document.querySelector("#reportTickRate").value = conf.reportTickRate;
          document.querySelector("#globalTickRate").value = conf.globalTickRate;
        });

      /**
       * Easy selector helper function
       */
      const select = (el, all = false) => {
        el = el.trim()
        if (all) {
          return [...document.querySelectorAll(el)]
        } else {
          return document.querySelector(el)
        }
      }

      /**
       * Easy event listener function
       */
      const on = (type, el, listener, all = false) => {
        let selectEl = select(el, all)
        if (selectEl) {
          if (all) {
            selectEl.forEach(e => e.addEventListener(type, listener))
          } else {
            selectEl.addEventListener(type, listener)
          }
        }
      }

      /**
       * Easy on scroll event listener
       */
      const onscroll = (el, listener) => {
        el.addEventListener('scroll', listener)
      }


      /**
       * Preloader
       */
      let preloader = select('#preloader');
      if (preloader) {
        window.addEventListener('load', () => {
          preloader.remove()
        });
      }

      function _(el) {
        return document.getElementById(el);
      }

      function showFiles(){
        _("detailsheader").innerHTML = "<h5>Files<h5>";
        fetch('./files')
          .then((response) => response.json())
          .then((msg) =>{
            let d = document.querySelector( "#details" );
        		d.innerHTML = '';
        		msg.data.files.forEach(e=>{
        			d.innerHTML += '<div class="d-flex border-bottom"><span class="bi-file-earmark-medical pe-2 flex-fill file-download" data-name="'+e.name+'">'+e.name+'</span><span class="text-end">'+e.size+'</span><span class="bi-trash3 file-delete" data-name="'+e.name+'"> </span></div>';
        		});
        		document.querySelectorAll('.file-delete').forEach((el)=>{el.addEventListener('click',()=>{
        			fetch('./file?name=/'+el.dataset.name+'&action=delete');
        			showFiles();
              _("status").innerHTML = "File deleted";
        		})});
        		document.querySelectorAll('.file-download').forEach((el)=>{el.addEventListener('click',()=>{
              fetch('./file?name=/'+el.dataset.name+'&action=download').then((res) => res.blob())
              .then((res) => {
                const aElement = document.createElement("a");
                aElement.setAttribute("download", el.dataset.name);
                const href = URL.createObjectURL(res);
                aElement.href = href;
                aElement.setAttribute("target", "_blank");
                aElement.click();
                URL.revokeObjectURL(href);
              });
              _("status").innerHTML = "File donwloaded";
        		})});
          });
      }

      document.querySelector("#filesModal").addEventListener('show.bs.modal', showFiles);

      function fileSelect(){
      	_("detailsheader").innerHTML = "<h3>Upload File<h3>"
        _("status").innerHTML = "";
        var uploadform ='<form id="upload_form" enctype="multipart/form-data" method="post">' +
        '<div class="mb-3">'+
        '<label for="file1" class="form-label btn btn-primary bi-search whiteF"> Select file to upload</label>'+
        '<input class="form-control" type="file" id="file1" onchange="uploadFile()">'+
        '</div>'+
        '<progress id="progressBar" value="0" max="100" class="w-100"></progress>' +
        '<p id="loaded_n_total"></p>' +
        '</form>';
        _("details").innerHTML = uploadform;
        _('file1').addEventListener('change',uploadFile);
      }
      function uploadFile() {
        var file = _("file1").files[0];
        // alert(file.name+" | "+file.size+" | "+file.type);
        var formdata = new FormData();
        formdata.append("file1", file);
        var ajax = new XMLHttpRequest();
        ajax.upload.addEventListener("progress", progressHandler, false);
        ajax.addEventListener("load", completeHandler, false); // doesnt appear to ever get called even upon success
        ajax.addEventListener("error", errorHandler, false);
        ajax.addEventListener("abort", abortHandler, false);
        ajax.open("POST", "/");
        ajax.send(formdata);
      }
      function progressHandler(event) {
        //_("loaded_n_total").innerHTML = "Uploaded " + event.loaded + " bytes of " + event.total; // event.total doesnt show accurate total file size
        _("loaded_n_total").innerHTML = "Uploaded " + event.loaded + " bytes";
        var percent = (event.loaded / event.total) * 100;
        _("progressBar").value = Math.round(percent);
        _("status").innerHTML = Math.round(percent) + "% uploaded... please wait";
        if (percent >= 100) {
          _("status").innerHTML = "Please wait, writing file to filesystem";
        }
      }
      function completeHandler(event) {
        _("status").innerHTML = "Upload Complete";
        showFiles();
        _("status").innerHTML = "File Uploaded";
      }
      function errorHandler(event) {
        _("status").innerHTML = "Upload Failed";
      }
      function abortHandler(event) {
        _("status").innerHTML = "inUpload Aborted";
      }

      document.querySelector('#reboot').addEventListener('click',()=>{location.href = location.origin+'/reboot';});

      document.querySelector('.modal-button-files').addEventListener('click',()=>{
        showFiles();
        document.querySelector('#filesModal').style.display="block";
        });
      document.querySelector('.modal-button-power').addEventListener('click',()=>{
        document.querySelector('#rebootModal').style.display="block";
        });

      document.querySelectorAll('.btn-close').forEach((el)=>{el.addEventListener('click',()=>{
        el.parentElement.parentElement.parentElement.parentElement.style.display="none";
        })});

    })()

  </script>
</body>
</html>
)rawliteral";

const char logout_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html lang="en">
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta charset="UTF-8">
</head>
<body>
  <p><a href="/">Log Back In</a></p>
</body>
</html>
)rawliteral";

// reboot.html base upon https://gist.github.com/Joel-James/62d98e8cb3a1b6b05102
const char reboot_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html lang="en">
<head>
  <meta charset="UTF-8">
</head>
<body>
  <h3>Rebooting, returning to main page in <span id="countdown">30</span> seconds</h3>
  <script type="text/javascript">
    var seconds = 20;
    function countdown() {
      seconds = seconds - 1;
      if (seconds < 0) {
        window.location = "/";
      } else {
        document.getElementById("countdown").innerHTML = seconds;
        window.setTimeout("countdown()", 1000);
      }
    }
    fetch("/reset");
    countdown();
  </script>
</body>
</html>
)rawliteral";

// Server configuration ############################################################################################################
void configureWebServer() {
  // if url isn't found
  server.onNotFound(notFound);

 #if MICRO_VERSION == 1
  // run handleUpload function when any file is uploaded
  server.onFileUpload(handleUpload);
 #endif

  // visiting this page will cause you to be logged out
  server.on("/logout", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(401);
  });

  // presents a "you are now logged out webpage
  server.on("/logged-out", HTTP_GET, [](AsyncWebServerRequest * request) {
    String logmessage = "Client:" + String(request->client()->remoteIP()) + " " + request->url();
    Serial.println(logmessage);
    request->send(401, "text/html", logout_html);
  });

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    if (!checkUserWebAuth(request)) return request->requestAuthentication();

     request->send(200, "text/html", index_html);
  });

  server.on("/configuration.json", HTTP_GET, [](AsyncWebServerRequest *request){
    if (!checkUserWebAuth(request)) return request->requestAuthentication();

    if(!db.fs->exists(db.configurationFile)){ // in case the html file does not exist get it from the in-memory resouce from html.h file
      server.onNotFound(notFound);
    }else{
      File file = db.open(db.configurationFile);
      request->send("text/html", file.size(), [&file](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {
        return file.read(buffer, maxLen);
      });
      file.close();
    }
  });

  server.on("/steerSettings.json", HTTP_GET, [](AsyncWebServerRequest *request){
    if (!checkUserWebAuth(request)) return request->requestAuthentication();

    if(!db.fs->exists(db.conf.steerSettingsFile)){ // in case the html file does not exist get it from the in-memory resouce from html.h file
      server.onNotFound(notFound);
    }else{
      File file = db.open(db.conf.steerSettingsFile);
      request->send("text/html", file.size(), [&file](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {
        return file.read(buffer, maxLen);
      });
      file.close();
    }
  });

  server.on("/steerConfiguration.json", HTTP_GET, [](AsyncWebServerRequest *request){
    if (!checkUserWebAuth(request)) return request->requestAuthentication();

    if(!db.fs->exists(db.conf.steerConfigurationFile)){ // in case the html file does not exist get it from the in-memory resouce from html.h file
      server.onNotFound(notFound);
    }else{
      File file = db.open(db.conf.steerConfigurationFile);
      request->send("text/html", file.size(), [&file](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {
        return file.read(buffer, maxLen);
      });
      file.close();
    }
  });

  server.on("/files", HTTP_GET, [](AsyncWebServerRequest *request){
    if (checkUserWebAuth(request)) {
      request->send(200, "text/javascript; charset=utf-8", listFiles(false));
    } else {
      return request->requestAuthentication();
    }
  });

  ArRequestHandlerFunction voR = [](AsyncWebServerRequest *request){};
  ArUploadHandlerFunction voU = [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {};
  // POST requests
  server.on("/save", HTTP_POST, voR, voU, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total){
    if (!checkUserWebAuth(request)) return request->requestAuthentication();

    if(db.webConfiguration(data)) request->send(201, "application/json", "{\"result\":\"ok\"}");
    else request->send(400, "application/json", "{\"error\":1}");
  });

  server.on("/", HTTP_POST, voR, handleUpload, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total){});

  server.on("/reboot", HTTP_GET, [](AsyncWebServerRequest * request) {
    if (!checkUserWebAuth(request)) return request->requestAuthentication();

    request->send(200, "text/html", reboot_html);
  });

  server.on("/reset", HTTP_GET, [](AsyncWebServerRequest * request) {
    //do reboot
    #if MICRO_VERSION == 1
      ESP.restart();
    #endif
    #if MICRO_VERSION == 2
      SCB_AIRCR = 0x05FA0004;
    #endif
  });

  server.on("/file", HTTP_GET, [](AsyncWebServerRequest * request) {
    if (!checkUserWebAuth(request)) return request->requestAuthentication();

    if (request->hasParam("name") && request->hasParam("action")) {
      const char *fileName = request->getParam("name")->value().c_str();
      const char *fileAction = request->getParam("action")->value().c_str();

      if (!db.fs->exists(fileName)) {
        request->send(400, "text/plain", "ERROR: file does not exist");
      } else {
        if (strcmp(fileAction, "download") == 0) {
          File file = db.open(fileName);
          request->send("application/octet-stream", file.size(), [&file](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {
            return file.read(buffer, maxLen);
          });
          file.close();
        } else if (strcmp(fileAction, "delete") == 0) {
          db.fs->remove(fileName);
          request->send(200, "text/plain", "Deleted File: " + String(fileName));
        } else {
          request->send(400, "text/plain", "ERROR: invalid action param supplied");
        }
      }
    } else {
      request->send(400, "text/plain", "ERROR: name and action params required");
    }
  });

}

void setServerMode() {
  Serial.println("Files in Memory:");
  Serial.println(listFiles(false));

 #if MICRO_VERSION == 1
  // init Soft Access Point on ESP32
  WiFi.mode(WIFI_MODE_AP); //https://techtutorialsx.com/2021/01/04/esp32-soft-ap-and-station-modes/
  while(!WiFi.softAP("WT32-AOG", "1234")){
    Serial.println(".");
    delay(100);
  }
  WiFi.softAPConfig(IPAddress(192,168,2,2), IPAddress(192,168,2,1), IPAddress(255,255,255,0));
  WiFi.begin("warhawk", "686654238");
  Serial.print("ESP32 IP as soft AP: ");
  Serial.println(WiFi.softAPIP());

  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.print("ESP32 IP on the WiFi network: ");
  Serial.println(WiFi.localIP());

  // info about network created
  Serial.println("\n\nNetwork Configuration:");
  Serial.println("----------------------");
  Serial.print("         SSID: "); Serial.println(WiFi.SSID());
  Serial.print("  Wifi Status: "); Serial.println(WiFi.status());
  Serial.print("Wifi Strength: "); Serial.print(WiFi.RSSI()); Serial.println(" dBm");
  Serial.print("          MAC: "); Serial.println(WiFi.macAddress());
  Serial.print("        AP IP: "); Serial.println(WiFi.softAPIP());
  Serial.print("           IP: "); Serial.println(WiFi.localIP());
  Serial.print("       Subnet: "); Serial.println(WiFi.subnetMask());
  Serial.print("      Gateway: "); Serial.println(WiFi.gatewayIP());
  Serial.print("        DNS 1: "); Serial.println(WiFi.dnsIP(0));
  Serial.print("        DNS 2: "); Serial.println(WiFi.dnsIP(1));
  Serial.print("        DNS 3: "); Serial.println(WiFi.dnsIP(2));
  Serial.println();
 #endif

  // configure web server
  Serial.println("Configuring Webserver ...");
  configureWebServer();

  // startup web server
  server.begin();
  Serial.println("HTTP server started");
}
#endif
