<!DOCTYPE html PUBLIC "-//W3C//DTD XHTML 1.0 Strict//EN" "http://www.w3.org/TR/xhtml1/DTD/xhtml1-strict.dtd">
<html>
<head>
<meta http-equiv="Content-Type" content="text/html; charset=utf-8">
<!--
/* ================== NOTIFICATION PAGE TEMPLATE: ERROR PAGES ================== */
/*  Edit and upload this template to customize your organization's error pages.  */
/*           Copyright © 2000-2010 Sophos Limited. All rights reserved.            */
/* ============================================================================= */
-->
<title>Sophos Web Appliance: Remote server not responding</title>

<style>
    html,body        	{ font-family: Arial, sans-serif; margin: 0; padding: 0; height: 99%; border: none; }
    .body,p            	{ font-size: 11px; color: #002F52; }
    .border01        	{ border: #dcdcdc solid 1px; background-color: #ebebeb; }
    .border02        	{ border: #ccc solid 1px; }

    .alertTitle {
	font-size: 14px; font-weight: 700; margin: 6px; padding: 10px 0 10px 40px;
	background-image:url(http://ws1100.swarthmore.edu//resources/images/ico_user_alerts.gif); background-repeat: no-repeat;
	background-position: 0 -286px;
    }

    .alertFooter {
	background-image:url(http://ws1100.swarthmore.edu//resources/images/wsa_alrt_foottile.jpg);
	color:#002F52;
	height:40px;
    }

    .mini { }

    .mini .alertFooter {
	display:none;
    }

    .mini img {
	display:none;
    }

    #envelope {
	width:500px;
    }

    .mini #envelope {
        width:100%;
    }
</style>

</head>
<body>
<div id="main" class="full" align="center">
        <!--  <img src='http://ws1100.swarthmore.edu//resources/dynamic/logo.jpg' />  This tag is OPTIONAL and uses the logo settings from the User Notification Options page -->
    <div id="envelope">
        <div class="border01" style="margin-top: 5px">
            <div class="border02">
                <div align="left" id="msg_title" class="alertTitle">
                    <h2>Website error:  Remote server not responding</h2> <!-- This tag is OPTIONAL and is the title eg. 404 - File not found -->
                </div>
                <div align="left" style="margin-left:40px;padding: 0 8px 8px 8px">
                    <p id="error_text">The remote server that hosts the web site or file you requested is currently not responding. The server may be overloaded or unavailable. Please click the back or reload button to attempt again.  If this error persists, you may want to try at a later time.</p> <!-- This tag is OPTIONAL and is the main text of the page. -->
                </div>
                <div id="footer" class="alertFooter">
                      <img align="left" src="http://ws1100.swarthmore.edu//resources/images/wsa_alrt_logo.jpg">
                </div>
            </div>
        </div>
    </div>
</div>
<script>
	if (self != top) {
	    document.getElementById('main').className = "nologo";
	    document.getElementById('footer').className = "nologo";
	}
</script> <!-- This tag is MANDATORY and must be located just before the closing body tag -->
</body>
</html>
