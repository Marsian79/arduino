<?php
	$cx=13;
	$cy=32;
	$cnum=15;
	
	$sx=3;
	$sy=32;
	$snum=2;
	
	
	$dxy=($cnum*$cx*$cy+$snum*$sx*$sy)/8;
	
	
	
	function drawc($c,$dx,$dy)
	{
			global $arri;
			echo "<div class='tw' data-c='{$c}' data-x='{$dx}' data-y='{$dy}' data-i='{$arri}'><span>{$c}</span><div class='t'>";
			$arri+=($dx*$dy)/8;
			
			for($y=0;$y<$dy;$y++)
			{
				echo "<div class='trow'>";
				for($x=0;$x<$dx;$x++) echo "<div class='tcell' data-xy='{$x}_{$y}'></div>";
				echo "</div>";
			}
			
			//echo "</div><input type='button' value='copy' onclick='copy(this)'><input type='button' value='del' onclick='del(this)'><input type='button' value='export' onclick='exp(this)'></div>";
			echo "</div><input type='button' value='copy' onclick='copy(this)'><input type='button' value='paste' onclick='paste(this)'><br><input type='button' value='clear' onclick='clr(this)'></div>";
	}
?>


<html>
	<head>
	<script src="https://code.jquery.com/jquery-3.2.1.min.js"></script>
		<style>
			.tw{display:inline-block;margin:5px;text-align:center;}
			.t{display:table;border-bottom:1px solid #aaa;border-left:1px solid #aaa;}
			.trow{display:table-row;}
			.tcell{display:table-cell;padding:3px;cursor:pointer;border-top:1px solid #aaa;border-right:1px solid #aaa;}
			.tcell:hover{background-color:grey;}
			.selected{background-color:black;}
		</style
	</head>
	<body>
		<div id="root">
		<input type='button' value='export' onclick='exp()'><input type='button' value='save' onclick='save()'><input type='button' value='load' onclick='load()'><br>
			<?php
				$arri=0;
				
				for($i=0;$i<10;$i++) drawc($i,$cx,$cy);
				echo "<br>";
				drawc('R',$cx,$cy);
				drawc('H',$cx,$cy);
				drawc('C',$cx,$cy);
				drawc('%',$cx,$cy);
				drawc('-',$cx,$cy);
				drawc('.',$sx,$sy);
				drawc("'",$sx,$sy);
			?>
		</div>
	<script>
		var s=false;
		var d=false;
		var curel;
		draw();
		
		
		
		$("body").keydown(function(event) {
			if ( event.which == 83 || event.which == 115) {event.preventDefault();s=true;}
			else if ( event.which == 68 || event.which == 100) {event.preventDefault();d=true;}
		});
		
		$("body").keyup(function(event) {
			if ( event.which == 83 || event.which == 115) {event.preventDefault();s=false;}
			else if ( event.which == 68 || event.which == 100) {event.preventDefault();d=false;}
		});
		
		function draw()
		{
			$(".tcell").off();
			$(".tcell").click(function(){
				if($(this).hasClass("selected")) {$(this).removeClass("selected");}
				else {$(this).addClass("selected");}
			});
			$( ".tcell" ).mouseenter(function() {
				if(s) {$(this).addClass("selected");}
				else if(d) {$(this).removeClass("selected");}
			});
		}
		
		/*function copy(el)
		{
			//$(el).parent().clone().appendTo("#root");
			$(el).parent().clone().insertAfter($(el).parent());
			draw();
		}
		function del(el)
		{
			$(el).parent().remove();
			draw();
		}*/
		
		function copy(el)
		{
			curel=el;
			
		}
		function paste(el)
		{
			$(curel).parent().find(".tcell").each(function(){
				var xy=$(this).data("xy");
				var s=$(this).hasClass("selected");
				$(el).parent().find("[data-xy='" + xy + "']").each(function(){
					if(s) {$(this).addClass("selected");}
					else {$(this).removeClass("selected");}
				})
			})
		}
		
		function clr(el)
		{
			//console.log(el);
			$(el).parent().find(".tcell").each(function(){
				$(this).removeClass("selected");
			})
		}
		
		
		
		
		var arr=new Array(<?=$dxy?>);
		
		function save()
		{
			savearr();
			var output=JSON.stringify(arr);
			console.log(output);
			
			$.ajax({
				url: "save.php",
				method: "POST",
				data: {arr:output}
			}).done(function() {
				alert("Saved");
			});
		}
		
		function savearr()
		{
			for(var i=0;i<<?=$dxy?>;i++){arr[i]=0;}
			
			$("#root").find(".tw").each(function(){
				saveel(this);
			});
		}
		
		
		function saveel(el)
		{
			var y=$(el).data("y");
			var i=$(el).data("i");
			$(el).find(".tcell.selected").each(function(){
				//if($(this).hasClass("selected")) {
					var xy=$(this).data("xy").split("_");
					var arrindex=Number(xy[0])*Number(y)+Number(xy[1]);
					var arrbyte=arrindex>>3;
					var arrbit=arrindex&0x0007;
					arr[arrbyte+Number(i)]|=1<<arrbit;
					
				//}
			});
		}
		
		function exp()
		{
			savearr();
			
			var arrhex=new Array(<?=$dxy?>);
			for(i=0;i<<?=$dxy?>;i++){
				var temp=arr[i].toString(16);
				if(temp.length==1) {temp='0'+temp;}
				arrhex[i]='0x'+temp;
			}
			var output=JSON.stringify(arrhex).replace(/\"/g,'');
			console.log(output);
			
		}
		
		
		
		function load()
		{
			$.ajax({
				url: "save.txt",
				method: "GET",
				
			}).done(function(result) {
				arr=JSON.parse(result);
				console.log(arr);
				$("#root").find(".tw").each(function(){
					loadel(this);
				});
				//alert("Loaded");
			});
			
			
		}
		
		function loadel(el)
		{
			var y=$(el).data("y");
			var i=$(el).data("i");
			//alert(i);
			$(el).find(".tcell").each(function(){
				
					var xy=$(this).data("xy").split("_");
					var arrindex=Number(xy[0])*Number(y)+Number(xy[1]);
					var arrbyte=arrindex>>3;
					var arrbit=arrindex&0x0007;
					if(arr[arrbyte+Number(i)]&(1<<arrbit)) {$(this).addClass("selected")};
					//alert(arrbyte+Number(i));
				
			});
		}
		
	</script>

	
	</body>
</html>